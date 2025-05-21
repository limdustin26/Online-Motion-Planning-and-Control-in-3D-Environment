#ifndef _A_STAR_H
#define _A_STAR_H

#include "navigation/data_struct.h"
#include "ros/node_handle.h"
#include "sensor_msgs/PointCloud2.h"
#include <Eigen/Eigen>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using namespace std;
using namespace Eigen;
class Astar {
public:
  Astar(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~Astar()= default;
  double getHeu(GridNodePtr node1, GridNodePtr node2);
  double getEuclidean(GridNodePtr node1, GridNodePtr node2);
  static double getManhHeu(GridNodePtr node1, GridNodePtr node2);
  void expandNode(GridNodePtr currentNode,
                  std::vector<GridNodePtr> &neighborSet,
                  std::vector<double> &fscoreSet);

  Eigen::Vector2d grid2coor(
      const Eigen::Vector2i
          &index); // Vector2d like a array, so use & as param is more effective
  Eigen::Vector2i coor2grid(const Eigen::Vector2d &positon);

  void AstarFinder(const Eigen::Vector2d& start_ps, const Eigen::Vector2d& end_ps);
  static void resetGrid(GridNodePtr node);

  void resetOccupancy(const int &x, const int &y);
  void resetGridNodemap();
  void resetLocalGridmap(const Eigen::Vector2d& local_min, const Eigen::Vector2d& local_max);
  // void resetLocalGridmap();
  void updateGridNodemap(const Vector2d& map_center);
  [[nodiscard]] bool isOccupied(const int &idx_x, const int &idx_y) const;
  [[nodiscard]] bool isOccupied(const Eigen::Vector2i &index) const;
  std::vector<Eigen::Vector2d> getPath();
  std::vector<Eigen::Vector2d> getVisitedNodes();
  void visGridPath(vector<Vector2d> nodes);
  void visVistedNodes(vector<Vector2d> nodes);
  void visGridMap();
  void setObs(double coord_x, double coord_y);
  double cost2Obs(GridNodePtr gridnode);
  void inflateObs();
  void inflateObs(const Eigen::Vector2i &local_min, const Eigen::Vector2i &local_max);
  Vector2d coordInRobotFrame(Vector2d coor) const;

  void visInflateMap();
  bool isInflated(const int &idx_x, const int &idx_y) const;

private:
  GridNodePtr **GridNodeMap; // a GridMap to put the GridNodePtr
  Eigen::Vector2i goalIdx_;
  GridNodePtr terminatePtr_;
  double map_width_, map_height_;
  Vector2d map_center_;

  int x_max_index_, y_max_index_;
  double time_brk_ ;
  double resolution_, inv_resolution_;
  double x_up_bound_, x_low_bound_, y_up_bound_, y_low_bound_;
  tf2_ros::Buffer tfBuffer_;  
  tf2_ros::TransformListener tfListener_;  // Needs to be declared **after** tfBuffer_
  Eigen::Vector2i local_min_, local_max_;

  ros::Publisher grid_path_vis_pub_, visited_nodes_vis_pub_,
      GridNodeMap_vis_pub_, Obs_vis_pub_, InflateMap_vis_pub_;
  // std::vector<GridNodePtr> pathGrid;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::multimap<double, GridNodePtr> openSet_;
};

#endif