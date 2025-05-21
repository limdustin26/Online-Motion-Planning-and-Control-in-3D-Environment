#include "navigation/safe_corridor_generator.h"

#include <utility>

#include "navigation/astar.h"
#include "navigation/data_struct.h"
#include "nav_msgs/Odometry.h"


SafeMoveCorridor::SafeMoveCorridor(
    const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh),
      nh_private_(pnh),
      x_low_bound_(0.0),
      x_up_bound_(0.0),
      y_low_bound_(0.0),
      y_up_bound_(0.0),
      x_map_index_(0),
      y_map_index_(0),
      iteration_num_(0),
      expand_step_(0),
      map_resolution_(0.05),
      max_acc_(0.0),
      max_vel_(0.0),
      astar_(nh, pnh) {
  {
    // map width, height , x,y bounds are w.r.t robot (robot is map center)
    nh_private_.param("map/width", map_width_, 20.0);
    nh_private_.param("map/height", map_height_, 20.0);
    nh_private_.param("map/resolution", map_resolution_, 0.05);
    
    nh_private_.param("retain_corridor_width", retain_corridor_width_, 10.0);
    nh_private_.param("retain_corridor_height", retain_corridor_height_, 10.0);

    nh_private_.param("planning/max_acc", max_acc_, 1.0);
    nh_private_.param("planning/max_vel", max_vel_, 1.0);
    nh_private_.param("planning/iteration_num", iteration_num_, 50);
    nh_private_.param("planning/expand_step", expand_step_, 1);
  }
  map_width_ = abs(map_width_);
  map_height_ = abs(map_height_);
  x_map_index_ = map_width_ / map_resolution_;
  y_map_index_ = map_height_ / map_resolution_;
  x_low_bound_ = map_width_ / -2.0;
  x_up_bound_ = map_width_ / 2.0;
  y_low_bound_ = map_height_ /-2.0;
  y_up_bound_ = map_height_ / 2.0;
  map_center_(0) = 0.0;
  map_center_(1) = 0.0;

  retain_corridor_x_up_ = retain_corridor_width_ / 2.0;
  retain_corridor_x_low_ = -retain_corridor_width_ / 2.0;
  retain_corridor_y_up_ = retain_corridor_height_ / 2.0;
  retain_corridor_y_low_ = -retain_corridor_height_ / 2.0;

  corridor_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "vis_corridor", 100);
  center_pt_publisher =
      nh_private_.advertise<visualization_msgs::Marker>("vis_center", 10);

  map_boundary_pub_ = nh_private_.advertise<visualization_msgs::Marker>("map_bound", 10);
  retain_corridor_boundary_pub_ = nh_private_.advertise<visualization_msgs::Marker>("retain_corridor_bound", 10);
  visRetainCorridorBoundary();
}

/*
    pari<Corridor,  bool>  record every Cube of waypoints
*/
pair<Cube, bool> SafeMoveCorridor::expandCube(Cube& newCube, Cube& lastCube) {
  Cube Cube_max = newCube;  // now newcube is a point
  MatrixXi vertex_idx(4, 2);
  MatrixXi vertex_idx_last(4, 2);
  MatrixXd vertex_coord(4, 2);
  Vector2d center_coord = newCube.center;
  Vector2i center_index = astar_.coor2grid(center_coord);
  bool collide_p1_p2 = false;
  bool collide_p2_p3 = false;
  bool collide_p3_p4 = false;
  bool collide_p1_p4 = false;
  if (isConstains(
          lastCube,
          Cube_max))  // 之前没加入iter=3的限制导致了生成的corrido太过稀疏，公共部分太小导致
    return make_pair(lastCube, false);

  if (astar_.isOccupied(center_index)) {
    ROS_ERROR("[Planning Node] path has node in obstacles !");
    return make_pair(Cube_max, false);
  }
  for (int i = 0; i < 4; i++) {
    vertex_idx.row(i) = center_index;
  }
  int idx_x_up, idx_x_lo, idx_y_up, idx_y_lo;
  int iter = 0;
  while (iter < iteration_num_) {
    // X , Y inflation
    idx_x_up = min(center_index(0) + iter + 1, x_map_index_ - 1);
    idx_x_lo = max(center_index(0) - (iter + 1), 0);
    idx_y_lo = max(center_index(1) - (iter + 1), 0);
    idx_y_up = min(center_index(1) + (iter + 1), y_map_index_ - 1);

    // check wheter collide

    // P1 - P2  Y+ direction expand direction
    if (!collide_p1_p2) {
      for (int i = vertex_idx(0, 1) + 1; i <= idx_y_up; ++i)  // y coordinate
      {
        for (int j = vertex_idx(0, 0); j <= vertex_idx(1, 0);
             ++j)  // x coordinate
        {
          if (astar_.isOccupied(j, i)) {
            collide_p1_p2 = true;
            break;
          }
        }
      }
      if (collide_p1_p2) {
        vertex_idx(0, 1) = idx_y_up - 1;
        vertex_idx(1, 1) = idx_y_up - 1;
      } else {
        vertex_idx(0, 1) = idx_y_up;
        vertex_idx(1, 1) = idx_y_up;
      }
    }

    // P4 -P3   Y- direction expands
    if (!collide_p3_p4) {
      for (int i = vertex_idx(3, 1) - 1; i >= idx_y_lo; --i) {
        for (int j = vertex_idx(3, 0); j <= vertex_idx(2, 0); ++j) {
          if (astar_.isOccupied(j, i)) {
            collide_p3_p4 = true;
            break;
          }
        }
      }
      if (collide_p3_p4) {
        vertex_idx(2, 1) = idx_y_lo + 1;
        vertex_idx(3, 1) = idx_y_lo + 1;
      } else {
        vertex_idx(2, 1) = idx_y_lo;
        vertex_idx(3, 1) = idx_y_lo;
      }
    }
    // P3 -P2 X+ direction
    if (!collide_p2_p3) {
      for (int i = vertex_idx(2, 0) + 1; i <= idx_x_up; ++i) {
        for (int j = vertex_idx(1, 1); j >= vertex_idx(2, 1); --j) {
          if (astar_.isOccupied(i, j)) {
            collide_p2_p3 = true;
            break;
          }
        }
      }
      if (collide_p2_p3) {
        vertex_idx(1, 0) = idx_x_up - 1;
        vertex_idx(2, 0) = idx_x_up - 1;
      } else {
        vertex_idx(1, 0) = idx_x_up;
        vertex_idx(2, 0) = idx_x_up;
      }
    }

    // P4 -P1 X- direction
    if (!collide_p1_p4) {
      for (int i = vertex_idx(0, 0) - 1; i >= idx_x_lo; --i) {
        for (int j = vertex_idx(0, 1); j >= vertex_idx(3, 1); --j) {
          if (astar_.isOccupied(i, j)) {
            collide_p1_p4 = true;
            break;
          }
        }
      }
      if (collide_p1_p4) {
        vertex_idx(0, 0) = idx_x_lo + 1;
        vertex_idx(3, 0) = idx_x_lo + 1;
      } else {
        vertex_idx(0, 0) = idx_x_lo;
        vertex_idx(3, 0) = idx_x_lo;
      }
    }
    // if the size of corrdidor doesn't change,  break from while loop
    if (vertex_idx_last == vertex_idx)
      break;

    vertex_idx_last = vertex_idx;
    // std::cout << "vertex_idx"  << vertex_idx << std::endl;
    for (int i = 0; i < 4; ++i) {
      int vertex_idx_x = vertex_idx(i, 0);
      int vertex_idx_y = vertex_idx(i, 1);

      Vector2i index(vertex_idx_x, vertex_idx_y);
      Vector2d coord = astar_.grid2coor(index);
      vertex_coord.row(i) = coord;
    }
    Cube_max.setVertex(vertex_coord, map_resolution_);
    if(isConstains(lastCube , Cube_max)&& iter == 3)
//     if(isConstains(lastCube , Cube_max))   //
////     之前没加入iter=3的限制导致了生成的corrido太过稀疏，公共部分太小导致
      return make_pair(lastCube , false);
    iter++;
  }
  return make_pair(Cube_max, true);
}

// old implementation
// void SafeMoveCorridor::simplify(vector<Cube>& corridor) {
//   Cube cube_self, cube;
//   auto temp = corridor;
//   int idx = 0;
//   int length = (int)temp.size();
//   corridor.clear();
//   corridor.push_back(temp[idx]);
//   std::cout << "push back cube: " << idx <<  std::endl;
//   for (int i = 1; i < length; ++i) {
//     cube_self = temp[idx];
//     cube = temp[i];
//     if (!isOverlap(cube_self, cube)) {
//       idx = i - 1;
//       corridor.push_back(temp[idx]);
//       std::cout << "push back cube: " << idx <<  std::endl;
//       // if(isOverlap(temp[idx] , temp[length - 1]))
//       // break;
//     }
//   }
//   corridor.push_back(temp[length - 1]);
//   std::cout << "push back cube: " << length - 1 <<  std::endl;
// }

void SafeMoveCorridor::simplify(vector<Cube>& corridor) {
  if (corridor.empty()){
  std::cout << "corridor empty ! cannot simplify" <<  std::endl;
      return;
  }
  vector<Cube> simplified;
  simplified.push_back(corridor[0]);
    std::cout << "push back cube: " << 0 <<  std::endl;

  int currentIndex = 0;
  int n = corridor.size();
  std::cout << "corridor size: " << n <<  std::endl;

  if(n == 1){
    simplified.push_back(corridor[0]);
    corridor = simplified;
    std::cout << "corridor only have 1 cube !" <<  std::endl;
    return;
  }

  while (currentIndex < n - 1) {
    std::cout << "currentIndex: " << currentIndex <<  std::endl;
      Cube currentCube = corridor[currentIndex];
      int bestIndex = currentIndex + 1;
      float bestOverlap = getOverlayArea(currentCube, corridor[bestIndex]);

      int j = currentIndex + 1;
      while (j < n && isOverlap(currentCube, corridor[j])) {
        std::cout << "j: " << currentIndex <<  std::endl;
          float overlap = getOverlayArea(currentCube, corridor[j]);
          // Update the best candidate if a larger overlap is found.
          if (overlap > bestOverlap) {
              bestOverlap = overlap;
              bestIndex = j;
              std::cout << "bestIndex: " << bestIndex <<  std::endl;
          }
          j++;
      }

      simplified.push_back(corridor[bestIndex]);
      std::cout << "push back cube: " << bestIndex <<  std::endl;
      currentIndex = bestIndex;
  }

  corridor = simplified;
}




// void SafeMoveCorridor::simplify(vector<Cube>& corridor) {
//   Cube cube_self, cube;
//   auto temp = corridor;
//   int idx = 0;
//   int overlay_idx = 0;
//   int length = (int)temp.size();
//   float overlay_area = 0.0;
//   corridor.clear();
//   corridor.push_back(temp[idx]);
//   std::cout << "corridor size: " << length << std::endl;
//   for (int i = 1; i < length + 1; ++i) {
//     if((i == length) && (i!=overlay_idx)){
//       std::cout << "pushed back last idx !" << idx << std::endl;
//       corridor.push_back(temp[idx]);
//       break;
//     }
//     cube_self = temp[idx];
//     cube = temp[i];
//     std::cout << "inspect corridor: " << idx << " with corridor: " << i << std::endl;

//     if(!isOverlap(cube_self,cube)){
//       std::cout << "inspect corridor: " << idx << " did not overlap with corridor: " << i << std::endl;
//       corridor.push_back(temp[overlay_idx]);
//       idx = overlay_idx;
//       cube_self = temp[idx];
//       std::cout << "pushed back overlay corridor: " << idx << std::endl;
//       overlay_area = 0.0;
//       for(int j = idx; j < i; j++){
//         if(idx == (i-1)){
//           std::cout << "already push back corridor: " << idx << std::endl;
//           continue;
//         }
//         if(j == (i-1)){
//           idx = j;
//           cube_self = temp[idx];
//           overlay_idx = j;
//           corridor.push_back(temp[overlay_idx]);
//       std::cout << "pushed back overlay corridor: " << overlay_idx << std::endl;
//         }
//         if(!isOverlap(cube_self,cube)){
//           std::cout << "inspect corridor: " << j << " did not overlap with corridor: " << i << std::endl;
//           if(j == overlay_idx){
//             idx = j+1;
//             cube_self = temp[idx];
//             continue;
//           }
//           idx = j;
//           cube_self = temp[idx] ;
//         }
//       }
//     }
    
//     std::cout << "inspect corridor: " << idx << " with corridor: " << i << std::endl;
//     float temp_area = getOverlayArea(cube_self, cube);
//     std::cout << "temp area: " << temp_area << std::endl;
//     if (temp_area > overlay_area) {
//         std::cout << "changed overlay area from: " << overlay_area << " to " << temp_area << std::endl;
//         overlay_area = temp_area;
//         overlay_idx = i;
//       }
//   std::cout << "current i: " << i << std::endl;
//   }
//   corridor.push_back(temp[length - 1]);
//   std::cout << "pushed back last corridor !" << std::endl;
// }

bool SafeMoveCorridor::isConstains(Cube& Cube1, Cube& Cube2) {
  if (Cube1.vertex(0, 0) <= Cube2.vertex(0, 0) &&
      Cube1.vertex(0, 1) >= Cube2.vertex(0, 1) &&
      Cube1.vertex(1, 0) >= Cube2.vertex(1, 0) &&
      Cube1.vertex(1, 1) >= Cube2.vertex(1, 1) &&
      Cube1.vertex(2, 0) >= Cube2.vertex(2, 0) &&
      Cube1.vertex(2, 1) <= Cube2.vertex(2, 1) &&
      Cube1.vertex(3, 0) <= Cube2.vertex(3, 0) &&
      Cube1.vertex(3, 1) <= Cube2.vertex(3, 1))
    return true;
  else
    return false;
}

bool SafeMoveCorridor::isOverlap(Cube& cube_self, Cube& cube) {
  // std::cout<<"cube_self 0,1: "<<cube_self.vertex(0, 1)<<std::endl;
  // std::cout<<"cube      2,1: "<<cube.vertex(2, 1)<<std::endl;
  // std::cout<<"cube_self 2,1: "<<cube_self.vertex(2, 1)<<std::endl;
  // std::cout<<"cube      0,1: "<<cube.vertex(0, 1)<<std::endl;
  // std::cout<<"cube_self 0,0: "<<cube_self.vertex(0, 0)<<std::endl;
  // std::cout<<"cube      2,0 "<<cube.vertex(2, 0)<<std::endl;
  // std::cout<<"cube_self 2,0: "<<cube_self.vertex(2, 0)<<std::endl;
  // std::cout<<"cube      0,0 "<<cube.vertex(0, 0)<<std::endl;
  if (cube_self.vertex(0, 1) <= cube.vertex(2, 1) ||
      cube_self.vertex(2, 1) >= cube.vertex(0, 1) ||
      cube_self.vertex(0, 0) >= cube.vertex(2, 0) ||
      cube_self.vertex(2, 0) <= cube.vertex(0, 0))
    return false;
  else
    return true;
}

float SafeMoveCorridor::getOverlayArea(Cube& cube1, Cube& cube2){
  // std::cout<<"cube1_x left: "<<cube1.vertex(1, 0)<<std::endl;
  // std::cout<<"cube2_x left: "<<cube2.vertex(1, 0)<<std::endl;
  // std::cout<<"cube1_x right: "<<cube1.vertex(0, 0)<<std::endl;
  // std::cout<<"cube2_x right: "<<cube2.vertex(0, 0)<<std::endl;
  // std::cout<<"cube1_y up: "<<cube1.vertex(0, 1)<<std::endl;
  // std::cout<<"cube2_y up: "<<cube2.vertex(0, 1)<<std::endl;
  // std::cout<<"cube1_y down: "<<cube1.vertex(3, 1)<<std::endl;
  // std::cout<<"cube2_y down: "<<cube2.vertex(3, 1)<<std::endl;
  float x_overlap = min(cube1.vertex(1, 0), cube2.vertex(1, 0)) - max(cube1.vertex(0, 0), cube2.vertex(0, 0));
  float y_overlap = min(cube1.vertex(0, 1), cube2.vertex(0, 1)) - max(cube1.vertex(3, 1), cube2.vertex(3, 1));
  // std::cout<<"x_overlap: "<<x_overlap<<std::endl;
  // std::cout<<"y_overlap: "<<y_overlap<<std::endl;
  return fabs(x_overlap * y_overlap);
}
/*
    初始传进去的pt是A*找到的waypoints的物理坐标
*/
Cube SafeMoveCorridor::generateCube(Vector2d pt) {
  Cube Cube;
  double x_u, x_l, y_u, y_l;
  x_u = x_l = pt(0);
  y_u = y_l = pt(1);
  Cube.vertex.row(0) = Vector2d(x_l, y_u);
  Cube.vertex.row(1) = Vector2d(x_u, y_u);
  Cube.vertex.row(2) = Vector2d(x_u, y_l);
  Cube.vertex.row(3) = Vector2d(x_l, y_l);
  Cube.center = pt;
  return Cube;
}

void SafeMoveCorridor::retainCorridor() {
  vector<Cube> temp = safe_corridor_;
  retain_safe_corridor_.clear();
  for (auto& cube : temp) {
    bool retain = false;
      if (cube.vertex(0, 0) >= retain_corridor_x_low_ && cube.vertex(0, 1) <= retain_corridor_y_up_ ||
          cube.vertex(1, 0) <= retain_corridor_x_up_ && cube.vertex(1, 1) <= retain_corridor_y_up_ ||
          cube.vertex(2, 0) <= retain_corridor_x_up_ && cube.vertex(2, 1) >= retain_corridor_y_low_ ||
          cube.vertex(3, 0) >= retain_corridor_x_low_ && cube.vertex(3, 1) >= retain_corridor_y_low_) {
        retain = true;
        break;
      }
    
    if (retain) {
      // Adjust cube vertices to fit within the boundary
      cube.vertex(0, 0) = std::max(retain_corridor_x_low_, std::min(cube.vertex(0, 0), retain_corridor_x_up_));
      cube.vertex(0, 1) = std::max(retain_corridor_y_low_, std::min(cube.vertex(0, 1), retain_corridor_y_up_));
      cube.vertex(1, 0) = std::max(retain_corridor_x_low_, std::min(cube.vertex(1, 0), retain_corridor_x_up_));
      cube.vertex(1, 1) = std::max(retain_corridor_y_low_, std::min(cube.vertex(1, 1), retain_corridor_y_up_));
      cube.vertex(2, 0) = std::max(retain_corridor_x_low_, std::min(cube.vertex(2, 0), retain_corridor_x_up_));
      cube.vertex(2, 1) = std::max(retain_corridor_y_low_, std::min(cube.vertex(2, 1), retain_corridor_y_up_));
      cube.vertex(3, 0) = std::max(retain_corridor_x_low_, std::min(cube.vertex(3, 0), retain_corridor_x_up_));
      cube.vertex(3, 1) = std::max(retain_corridor_y_low_, std::min(cube.vertex(3, 1), retain_corridor_y_up_));
      retain_safe_corridor_.push_back(cube);
    }
  }
}

void SafeMoveCorridor::genCorridor(
    const Vector2d& start_ps, const Vector2d& target_ps,
    const Vector2d& start_vel, const vector<Vector2d>& path) {
  if (!safe_corridor_.empty()) {
    // retainCorridor();
    safe_corridor_.clear();
    for (auto& mk : cube_vis.markers)
      mk.action = visualization_msgs::Marker::DELETE;  // 删除上一次的cube
    cube_vis.markers.clear();
  }

  Cube lastCube, newCube;

  Vector2d waypoint;
  // std::cout << path.size() <<std::endl;
  for (int i = 0; i < (int)path.size(); ++i) {
    waypoint = path[i];
    newCube = generateCube(waypoint);
    auto is_saveCube = expandCube(newCube, lastCube);
    if (!is_saveCube.second)
      continue;
    // std::cout << "waypoints is :" << waypoit << std::endl;
    newCube = is_saveCube.first;
    // std::cout << "Cube center is :" << newCube.center << std::endl;
    lastCube = newCube;
    safe_corridor_.push_back(newCube);
  }
  simplify(safe_corridor_);
  std::cout<<"done simplify"<<std::endl;
  // simplify(safe_corridor_, retain_safe_corridor_);
  visCorridor(safe_corridor_);
  std::cout<<"done vis corridor"<<std::endl;
  vector<Vector2d> corridor_center_list = getCenterPt();
  std::cout<<"done get center pt"<<std::endl;
  timeAllocation(start_ps, target_ps, start_vel, corridor_center_list);
  std::cout<<"done time alloc"<<std::endl;
  visCenter(corridor_center_list);
  std::cout<<"done vis center"<<std::endl;

}

vector<Vector2d> SafeMoveCorridor::getCenterPt() {
  vector<Vector2d> center_pt_list;
  Vector2d center_pt;
  if(safe_corridor_.empty()){
    ROS_WARN("the safe corridor generates fail!..");
    return {};
  }
  for (int i = 0; i < (int)safe_corridor_.size() - 1; ++i) {
    double x_low =
        max(safe_corridor_[i].vertex(0, 0), safe_corridor_[i + 1].vertex(0, 0));
    double x_up =
        min(safe_corridor_[i].vertex(2, 0), safe_corridor_[i + 1].vertex(2, 0));
    double y_low =
        max(safe_corridor_[i].vertex(2, 1), safe_corridor_[i + 1].vertex(2, 1));
    double y_up =
        min(safe_corridor_[i].vertex(0, 1), safe_corridor_[i + 1].vertex(0, 1));
    center_pt(0) = (x_low + x_up) / 2.0;
    center_pt(1) = (y_low + y_up) / 2.0;
    center_pt_list.push_back(center_pt);

    // std::cout << "center_pt_x :" << center_pt(0) << std::endl;
    // std::cout << "center_pt_y :" << center_pt(1) << std::endl;
  }
  return center_pt_list;
}

void SafeMoveCorridor::visMapBoundary(){
   visualization_msgs::Marker marker;

    // Configure marker header
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();

    // Set marker properties
    marker.ns = "map_boundary";
    marker.id = 0;  // Single marker, fixed ID
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;  // Line width
    marker.color.r = 1.0;  // Red color
    marker.color.a = 1.0;  // Full opacity

    // Create boundary points (closed rectangle)
    geometry_msgs::Point p;
    std::vector<std::pair<double, double>> vertices = {
        {x_low_bound_, y_low_bound_},
        {x_up_bound_, y_low_bound_},
        {x_up_bound_, y_up_bound_},
        {x_low_bound_, y_up_bound_},
        {x_low_bound_, y_low_bound_}  // Close the loop
    };

    // Add points to marker
    for (const auto& vertex : vertices) {
        p.x = vertex.first;
        p.y = vertex.second;
        p.z = 0;
        marker.points.push_back(p);
    }

    map_boundary_pub_.publish(marker);  // Directly publish the single marker

}

void SafeMoveCorridor::visRetainCorridorBoundary(){
  visualization_msgs::Marker marker;

   // Configure marker header
   marker.header.frame_id = "odom";
   marker.header.stamp = ros::Time::now();

   // Set marker properties
   marker.ns = "retain_corridor_boundary";
   marker.id = 0;  // Single marker, fixed ID
   marker.type = visualization_msgs::Marker::LINE_STRIP;
   marker.action = visualization_msgs::Marker::ADD;
   marker.scale.x = 0.1;  // Line width
   marker.color.b = 1.0;  // Red color
   marker.color.a = 1.0;  // Full opacity

   // Create boundary points (closed rectangle)
   geometry_msgs::Point p;
   std::vector<std::pair<double, double>> vertices = {
       {retain_corridor_x_low_, retain_corridor_y_low_},
       {retain_corridor_x_up_, retain_corridor_y_low_},
       {retain_corridor_x_up_, retain_corridor_y_up_},
       {retain_corridor_x_low_, retain_corridor_y_up_},
       {retain_corridor_x_low_, retain_corridor_y_low_}  // Close the loop
   };

   // Add points to marker
   for (const auto& vertex : vertices) {
       p.x = vertex.first;
       p.y = vertex.second;
       p.z = 0;
       marker.points.push_back(p);
   }

   retain_corridor_boundary_pub_.publish(marker);  // Directly publish the single marker

}


void SafeMoveCorridor::visCorridor(vector<Cube>& SafeMovingCorridor) {
  for (auto& mk : cube_vis.markers)
    mk.action = visualization_msgs::Marker::DELETE;  // 删除上一次的cube

  cube_vis.markers.clear();

  visualization_msgs::Marker mk;
  mk.header.frame_id = "odom";
  // mk.header.frame_id = "camera_init";
  mk.header.stamp = ros::Time::now();
  mk.ns = "corridor";
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;

  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.a = 0.6;
  mk.color.r = 0.0;
  mk.color.g = 1.0;
  mk.color.b = 1.0;

  int idx = 0;
  std::cout << SafeMovingCorridor.size() << std::endl;
  for (int i = 0; i < SafeMovingCorridor.size(); ++i) {
    mk.id = i;

    mk.pose.position.x = (SafeMovingCorridor[i].vertex(0, 0) +
                          SafeMovingCorridor[i].vertex(1, 0)) /
                         2.0;
    mk.pose.position.y = (SafeMovingCorridor[i].vertex(0, 1) +
                          SafeMovingCorridor[i].vertex(3, 1)) /
                         2.0;
    mk.pose.position.z = 0.0;  // 二维

    mk.scale.x =
        (SafeMovingCorridor[i].vertex(1, 0) -
         SafeMovingCorridor[i].vertex(0, 0));
    mk.scale.y =
        (SafeMovingCorridor[i].vertex(0, 1) -
         SafeMovingCorridor[i].vertex(3, 1));
    mk.scale.z = 0.05;

    idx++;
    cube_vis.markers.push_back(mk);
  }
  corridor_pub_.publish(cube_vis);
}
void SafeMoveCorridor::visCenter(vector<Vector2d>& center_pt_list) {
  if(center_pt_list.empty()){
    ROS_WARN("the center point list is empty!..");
    return;
  }
  visualization_msgs::Marker center_vis;
  center_vis.header.frame_id = "odom";
  // center_vis.header.frame_id = "camera_init";
  center_vis.header.stamp = ros::Time::now();

  center_vis.type = visualization_msgs::Marker::CUBE_LIST;
  center_vis.action = visualization_msgs::Marker::ADD;
  center_vis.id = 0;

  center_vis.pose.orientation.x = 0.0;
  center_vis.pose.orientation.y = 0.0;
  center_vis.pose.orientation.z = 0.0;
  center_vis.pose.orientation.w = 1.0;

  center_vis.color.a = 1.0;
  center_vis.color.r = 1.0;
  center_vis.color.g = 0.0;
  center_vis.color.b = 0.0;

  center_vis.scale.x = 0.05;
  center_vis.scale.y = 0.05;
  center_vis.scale.z = 0.05;

  geometry_msgs::Point pt;
  int length = (int)center_pt_list.size();
  for (int i = 0; i < length; ++i) {
    Vector2d coord = center_pt_list[i];
    // std::cout << "nodes[i]" <<  nodes[i] << std::endl;
    pt.x = coord(0);
    pt.y = coord(1);
    pt.z = 0;

    center_vis.points.push_back(pt);
  }
  center_pt_publisher.publish(center_vis);
}

void SafeMoveCorridor::timeAllocation(
    const Vector2d& start_ps, const Vector2d& target_ps,
    const Vector2d& start_vel, const vector<Vector2d>& centerPtList) {
  vector<Vector2d> points;
  int size = static_cast<int>(centerPtList.size());
  double v_max = max_vel_;
  double a_max = max_acc_;
  double v_start = start_vel.norm();
  double acct = v_max / a_max;
  double dcct = v_max / a_max;
  double act_dist = 0.0;
  std::cout<<"done init"<<std::endl;
  std::cout<<"size: "<<size<<std::endl;
  std::cout<<"centerPtList[0]"<<centerPtList[0]<<std::endl;

  Vector2i firstPindex = astar_.coor2grid(centerPtList[0]);
  std::cout<<"done firstPindex"<<std::endl;
  Vector2i startPindex = astar_.coor2grid(start_ps);
  std::cout<<"done startPindex"<<std::endl;
  std::cout<<"I think below me is the problem"<<std::endl;
  Vector2i endPindex = astar_.coor2grid(centerPtList[size - 1]);
  std::cout<<"this is not printed"<<std::endl;
  Vector2i lastPindex = astar_.coor2grid(target_ps);
  points.push_back(start_ps);

  for (int i = 0; i < (int)centerPtList.size(); i++)
    points.push_back(centerPtList[i]);

  points.push_back(target_ps);

  std::cout<<"done push back points"<<std::endl;

  double dist;
  for (int k = 0; k < (int)points.size() - 1; k++) {
    double t;
    Vector2d p0 = points[k];
    Vector2d p1 = points[k + 1];
    Vector2d d = p1 - p0;
    dist = d.norm();
    if (k == 0 & v_start != 0) {
      acct = (v_max - v_start) / a_max;
      act_dist = (v_start + v_max) * acct * 0.5 + v_max * dcct * 0.5;
    } else {
      act_dist = (acct + dcct) * v_max * 0.5;
    }
    if (act_dist <= dist)
      t = (dist - act_dist) / v_max + acct + dcct;
    else
      t = acct + dcct;
    safe_corridor_[k].t = t;
  }
}

// bool SafeMoveCorridor::IsTargetValid(double x, double y) const {
//   Vector2d coord(x,y);
//   Vector2d coordRobotFrame = astar_.coordInRobotFrame(coord);
//   if (coordRobotFrame(0) < x_low_bound_ || coordRobotFrame(0) > x_up_bound_ || coordRobotFrame(1) < y_low_bound_ ||
//       coordRobotFrame(1) > y_up_bound_)
//     return false;
//   return true;
// }

void SafeMoveCorridor::updateMapBoundary(const Vector2d& map_center) {
  astar_.updateGridNodemap(map_center);
  map_center_ = map_center;
  x_low_bound_ = map_center_(0) + map_width_ / -2.0;
  x_up_bound_ = map_center_(0) + map_width_ / 2.0;
  y_low_bound_ = map_center_(1) + map_height_ / -2.0;
  y_up_bound_ = map_center_(1) + map_height_ / 2.0;

}

bool SafeMoveCorridor::IsTargetValid(double x, double y) const {
  ROS_INFO("[isTargetValid] x: %f, y: %f", x, y);
  Vector2i grid;
  grid(0) = min(max(int((x - map_center_(0)) / map_resolution_), 0),
                 x_map_index_ - 1);
  // std::cout << "index(0)" <<index(0) << std::endl;
  grid(1) = min(max(int((y - map_center_(1)) / map_resolution_), 0),
                 y_map_index_ - 1);
  ROS_INFO("[isTargetValid] grid_x: %d, grid_y: %d", grid(0), grid(1));
  if (x < x_low_bound_ || x > x_up_bound_ || y < y_low_bound_ ||
      y > y_up_bound_)
    return false;
  return true;
}

void SafeMoveCorridor::setObs(double x, double y) {
  astar_.setObs(x, y);
}

void SafeMoveCorridor::updateLocalMap(const Eigen::Vector2d& local_min,
                                      const Eigen::Vector2d& local_max) {
  astar_.resetLocalGridmap(local_min, local_max);
}


void SafeMoveCorridor::frontEndPathFinder(
    const Vector2d& start_ps, const Vector2d& target_ps,
    const Vector2d& start_vel) {
  visMapBoundary();
  astar_.AstarFinder(start_ps, target_ps);
  auto grid_path = astar_.getPath();
  astar_.visGridPath(grid_path);
  astar_.resetGridNodemap();
  ros::Time time_before_gen_corridor = ros::Time::now();
  genCorridor(start_ps, target_ps, start_vel, grid_path);
  ros::Time time_after_gen_corridor = ros::Time::now();
  ROS_WARN(
      "Time consume in corridor generation is %f  s",
      (time_after_gen_corridor - time_before_gen_corridor).toSec());
}

bool SafeMoveCorridor::isOccupy(const Vector2i& index) {
  return astar_.isOccupied(index);
}

Vector2i SafeMoveCorridor::getGridmapIndex(const Vector2d& coor) {
  return astar_.coor2grid(coor);
}

void SafeMoveCorridor::visGridMap() {
  astar_.visGridMap();
}
void SafeMoveCorridor::inflateObs() {
  astar_.inflateObs();
}