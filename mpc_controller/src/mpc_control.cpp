#include "mpc_controller/mpc_control.hpp"

MPCController::MPCController()
{
    state_dim_ = 3;
    control_dim_ = 2;

    // // Initialize MPC
    // initMPC();

}

MPCController::~MPCController() {
  // Clear vectors to release memory
  q_.clear();
  l_.clear();
  u_.clear();

  // Resize matrices to release memory (Eigen will free memory)
  P_.resize(0, 0);
  C_.resize(0, 0);
}

void MPCController::initMPC(){
    // init matrices
    P_.resize(total_dim_, total_dim_);
    C_.resize(total_dim_ + total_state_dim_, total_dim_);
    q_.assign(total_dim_, 0.0);
    l_.assign(total_dim_ + total_state_dim_, 0.0);
    u_.assign(total_dim_ + total_state_dim_, 0.0);
    // Quadratic costs
    // P = diag(Q, Q,..,QN,R,..,R]
    std::vector<Triplet<double>> P_triplets;
    // Q
    for (int i=0; i < horizon_length_ ; i++)
    { 
        int local_row_offset = i*state_dim_;
        P_triplets.emplace_back(local_row_offset, local_row_offset , longitudinal_state_cost_);
        P_triplets.emplace_back(local_row_offset + 1, local_row_offset + 1, lateral_state_cost_);
        P_triplets.emplace_back(local_row_offset + 2, local_row_offset + 2 , orientation_state_cost_);
    }

    // QN
    P_triplets.emplace_back(horizon_length_*state_dim_, horizon_length_*state_dim_, terminal_state_cost_);
    P_triplets.emplace_back(horizon_length_*state_dim_ + 1, horizon_length_*state_dim_ + 1, terminal_state_cost_);
    P_triplets.emplace_back(horizon_length_*state_dim_ + 2, horizon_length_*state_dim_ + 2, terminal_state_cost_);
    
    // R
    for (int i = 0; i< horizon_length_; i++)
    {
        int local_row_offset = total_state_dim_ + i*control_dim_;
        P_triplets.emplace_back(local_row_offset, local_row_offset, linear_control_cost_);
        P_triplets.emplace_back(local_row_offset+ 1, local_row_offset+ 1, angular_control_cost_);
    }

    for (const auto& triplet : P_triplets) {
      if (triplet.row() < 0 || triplet.row() >= P_.rows() ||
          triplet.col() < 0 || triplet.col() >= P_.cols()) {
          std::cerr << "Invalid P_triplet: row=" << triplet.row()
                    << ", col=" << triplet.col()
                    << ", value=" << triplet.value()
                    << ", P_.rows=" << P_.rows()
                    << ", P_.cols=" << P_.cols() << std::endl;
      }
  }
  
  
    P_.setFromTriplets(P_triplets.begin(), P_triplets.end());
    P_.makeCompressed();

    // constraints
    // lineq, uineq
    // lineq starts from 2 * (state_dim_*(horizon_length_ + 1))
    int lineq_state_row_offset = total_state_dim_;
    for (int i = 0; i < total_state_dim_ ; i++)
    {
        int local_row_offset = lineq_state_row_offset + i;
        l_[local_row_offset] = -1 * std::numeric_limits<c_float>::infinity();
        u_[local_row_offset] = std::numeric_limits<c_float>::infinity();
        // l_[local_row_offset] = -25.0;
        // u_[local_row_offset] = 20.0;
    }

    int lineq_control_row_offset = 2*total_state_dim_;
    for (int i = 0; i < horizon_length_ ; i++)
    {
        int local_row_offset = lineq_control_row_offset + i*control_dim_;
        l_[local_row_offset] = -linear_vel_max_;
        l_[local_row_offset + 1] = -angular_vel_max_;
        u_[local_row_offset] = linear_vel_max_;
        u_[local_row_offset + 1] = angular_vel_max_;
    }

    // C
    // Ceq = [Ax, Bu], A reserve 5, B reserve 3, Identity matrix in Ax reserve 3, Cineq reserve total_dim_
    C_.reserve((5 + 3)*total_state_dim_ + 3*state_dim_*horizon_length_ + total_dim_);
    vector<Triplet<double>> C_triplets;
    // C_eq
    for (int i = 0; i< horizon_length_ + 1; i++)
    {
        // Ax (row == col), A and B row offset is the same
        // I
        int local_row_offset = i*state_dim_;
        int local_A_row_offset = local_row_offset + state_dim_;
        int global_B_col_offset = total_state_dim_;

        C_triplets.emplace_back(local_row_offset , local_row_offset, -1.0);
        C_triplets.emplace_back(local_row_offset + 1, local_row_offset +1, -1.0);
        C_triplets.emplace_back(local_row_offset + 2, local_row_offset +2, -1.0);
        
        if (i == horizon_length_){
            break;
        }
        // -A below -I
        C_triplets.emplace_back(local_A_row_offset , local_row_offset, 1.0);
        C_triplets.emplace_back(local_A_row_offset + 1, local_row_offset +1, 1.0);
        C_triplets.emplace_back(local_A_row_offset + 2, local_row_offset +2, 1.0);
        // Bu
        // first row of Bu is zero
        C_triplets.emplace_back(local_A_row_offset + 2,  global_B_col_offset + i * control_dim_ + 1, sample_t_);

        // Add placeholder entries for elements modified in updateReferenceState
        C_triplets.emplace_back(local_A_row_offset, local_row_offset + 2, 0.0);
        C_triplets.emplace_back(local_A_row_offset + 1, local_row_offset + 2, 0.0);
        C_triplets.emplace_back(local_A_row_offset, global_B_col_offset + i * control_dim_, 0.0);
        C_triplets.emplace_back(local_A_row_offset + 1, global_B_col_offset + i * control_dim_, 0.0);
    }

    // Cineq
    int cineq_row_offset = total_state_dim_;
    for (int i = 0; i< total_dim_  ; i++)
    { 
        C_triplets.emplace_back(cineq_row_offset + i, i, 1.0);
    }

    C_.setFromTriplets(C_triplets.begin(), C_triplets.end());

    setErrorSumZero();

    std::cout << "initialized MPC!" << std::endl;
}


void MPCController::setparams(int horizon_length, double sample_t, double longitudinal_state_cost, double lateral_state_cost,double orientation_state_cost, double terminal_state_cost, 
                              double linear_control_cost, double angular_control_cost, double linear_vel, double angular_vel){
  horizon_length_ = horizon_length;
  sample_t_ = sample_t;
  longitudinal_state_cost_ = longitudinal_state_cost;
  lateral_state_cost_ = lateral_state_cost;
  orientation_state_cost_ = orientation_state_cost;
  terminal_state_cost_ = terminal_state_cost;
  linear_control_cost_ = linear_control_cost;
  angular_control_cost_ = angular_control_cost;
  // angular_vel_max_ = linear_vel/min_turning_radius;
  angular_vel_max_ = angular_vel;
  linear_vel_max_ = linear_vel;

  total_state_dim_ = state_dim_*(horizon_length_ + 1);
  total_control_dim_ = control_dim_*horizon_length_;
  total_dim_ = total_state_dim_ + total_control_dim_;
}

int MPCController::find_nearest_pt(vector<Vector4d> &ref, Vector4d current_ps) {
  int min_idx = 0;
  double min_dist = sqrt(pow((ref[0](0) - current_ps(0)), 2) +
                         pow((ref[0](1) - current_ps(1)), 2));
  for (int i = 1; i < ref.size(); ++i) {

    double dist = sqrt(pow((ref[i](0) - current_ps(0)), 2) +
                       pow((ref[i](1) - current_ps(1)), 2));
    if (dist < min_dist) {
      min_idx = i;
      min_dist = dist;
    }
  }
  return min_idx;
}

int MPCController::find_nearest_pt(vector<Vector4d> &ref, Vector4d current_ps, int current_idx) {
  int min_idx = current_idx;
  double max_dist  = 1.0;
  double min_dist_thres = 0.005;
  for (int i = min_idx; i < ref.size(); ++i) {

    double dist = sqrt(pow((ref[i](0) - current_ps(0)), 2) +
                       pow((ref[i](1) - current_ps(1)), 2));
    if (dist < max_dist && dist >= min_dist_thres) {
      if(i >= min_idx){
            min_idx = i;
      }
      // min_dist = dist;
    }
  }

  return min_idx;
}

double MPCController::normalizeAngle(double angle) {
  angle = fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0) angle += 2.0 * M_PI;
  return angle - M_PI;
}

// update Ceq, leq, ueq, q
void MPCController::updateReferenceState(Vector4d ref_state) {
  std::cout<<"ref_state x: "<<ref_state(0)<<std::endl;
  double yaw = normalizeAngle(ref_state(2));
  std::cout<<"yaw ref: "<<yaw<<std::endl;
  // q
  double q_data0 = -longitudinal_state_cost_ * ref_state(0);
  double q_data1 = -lateral_state_cost_ * ref_state(1);
  double q_data2 = -orientation_state_cost_ * ref_state(2);
  for (int i = 0; i < horizon_length_ ; ++i) {
    int local_row_offset = i*state_dim_;
    q_[local_row_offset] = q_data0; 
    q_[local_row_offset + 1] = q_data1;
    q_[local_row_offset + 2] = q_data2;
  }
  q_[horizon_length_*state_dim_] = -terminal_state_cost_ * ref_state(0);
  q_[horizon_length_*state_dim_ + 1] = -terminal_state_cost_ * ref_state(1);
  q_[horizon_length_*state_dim_ + 2] = -terminal_state_cost_ * ref_state(2);
  
  std::cout << "updated q!" << std::endl;

  // C
  for(int i=0; i<horizon_length_ +1; i++){
    int local_row_offset = i*state_dim_;
    int local_A_row_offset = local_row_offset + state_dim_;
    int global_B_col_offset = total_state_dim_;

    if (i == horizon_length_){
      break;
  }

    C_.coeffRef(local_A_row_offset, local_row_offset + 2) = -ref_state(3) * sin(yaw) * sample_t_;
    C_.coeffRef(local_A_row_offset + 1, local_row_offset + 2) = ref_state(3) * cos(yaw) * sample_t_;
    C_.coeffRef(local_A_row_offset,  global_B_col_offset + i * control_dim_) = cos(yaw) * sample_t_;
    C_.coeffRef(local_A_row_offset + 1,  global_B_col_offset + i * control_dim_) = sin(yaw) * sample_t_;
  }
  C_.makeCompressed();
    std::cout << "updated C!" << std::endl;
  // }

}

void MPCController::updateInitialState(Vector4d cur_state) {
  double prev_x_init = l_[0];
  double prev_y_init = l_[1];
  double prev_theta_init = normalizeAngle(l_[2]);

  // std::cout<< "prev_x_init: "<<prev_x_init<<std::endl;
  // std::cout<< "cur_x: "<<-cur_state(0)<<std::endl;
  // std::cout<<" cur_x - prev_x: " <<std::abs(-cur_state(0) - prev_x_init)<<std::endl;

  // Only update if the new values are significantly different
  if (std::abs(-cur_state(0) - prev_x_init) > 0.01) {
      l_[0] = -cur_state(0);
      u_[0] = -cur_state(0);
  }
  if (std::abs(-cur_state(1) - prev_y_init) > 0.01) {
    l_[1] = -cur_state(1);
    u_[1] = -cur_state(1);
  }
  if (std::abs(-normalizeAngle(cur_state(2)) - prev_theta_init) > 0.01) {
    l_[2] = -normalizeAngle(cur_state(2));
    u_[2] = -normalizeAngle(cur_state(2));
  }
  // // Equality constraint: x0 == current_state
  // l_[0] = cur_state(0);  // Lower bound = current x
  // u_[0] = cur_state(0);  // Upper bound = current x
  // l_[1] = cur_state(1);
  // u_[1] = cur_state(1);
  // l_[2] = cur_state(2);
  // u_[2] = cur_state(2);
  std::cout << "updated initial state!" << std::endl;
}

void MPCController::FreeData(OSQPData* data) {
  delete[] data->q;
  delete[] data->l;
  delete[] data->u;

  delete[] data->P->i;
  delete[] data->P->p;
  delete[] data->P->x;

  delete[] data->A->i;
  delete[] data->A->p;
  delete[] data->A->x;
}

csc* MPCController::convertEigenToCSC(const Eigen::SparseMatrix<double>& mat) {
  // Get the correct StorageIndex type from the Eigen sparse matrix
  typedef typename Eigen::SparseMatrix<double>::StorageIndex SparseIndexType;

  c_int nzmax = static_cast<c_int>(mat.nonZeros());
  c_int rows = static_cast<c_int>(mat.rows());
  c_int cols = static_cast<c_int>(mat.cols());

  csc* cscMat = csc_matrix(rows, cols, nzmax,
      (c_float*)malloc(nzmax * sizeof(c_float)),
      (c_int*)malloc(nzmax * sizeof(c_int)),
      (c_int*)malloc((cols + 1) * sizeof(c_int))
  );

  // Copy values
  std::copy(mat.valuePtr(), mat.valuePtr() + nzmax, cscMat->x);

  // Convert and copy row indices
  const SparseIndexType* eigenInner = mat.innerIndexPtr();
  for (c_int i = 0; i < nzmax; ++i) {
    cscMat->i[i] = static_cast<c_int>(eigenInner[i]);
  }

  // Convert and copy column pointers
  const SparseIndexType* eigenOuter = mat.outerIndexPtr();
  for (c_int i = 0; i <= cols; ++i) {
    cscMat->p[i] = static_cast<c_int>(eigenOuter[i]);
  }

  return cscMat;
}


int MPCController::optimize(){
  // OSQP Solver setting
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  OSQPWorkspace* work;
  OSQPSettings* settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
  size_t kernal_dim = total_dim_;
  size_t cons_dim = total_dim_ + total_state_dim_ ;//constraint dimension
  data->n = kernal_dim;
  data->m = cons_dim;

  data->P = convertEigenToCSC(P_);

  data->A = convertEigenToCSC(C_);
  data->q = CopyData(q_);
  data->l = CopyData(l_);
  data->u = CopyData(u_);

//   std::cout << "Lower bounds (l): ";
// for (int i = 0; i < data->m; i++) std::cout << data->l[i] << " ";
// std::cout << std::endl;

// std::cout << "Upper bounds (u): ";
// for (int i = 0; i < data->m; i++) std::cout << data->u[i] << " ";
// std::cout << std::endl;


  // Solve Problem
  ros::Time before_sol = ros::Time::now();
  c_int exitflag = 0;
  // if (settings)
  osqp_set_default_settings(settings);
  settings->verbose = true;
  settings->polish = true;

  // Setup workspace
  exitflag = osqp_setup(&work, data, settings);

  // // Solve problem
  osqp_solve(work);

  auto status = work->info->status_val;  // whether solve succesfully

  // solve fail
  if (status < 0 || (status != 1 && status != 2)) {
    osqp_cleanup(work);
    FreeData(data);
    c_free(settings);
    return -1;
  } else if (work->solution == nullptr) {
    osqp_cleanup(work);
    FreeData(data);
    c_free(settings);
    return -1;
  }
  // solve success
  ros::Time time_end = ros::Time::now();
  ROS_WARN("time consume in optimize is :");
  std::cout << (time_end - before_sol) << std::endl;

  // get control input at first time step
  linear_vel_sol_ =  work->solution->x[total_state_dim_];  
  angular_vel_sol_ = work->solution->x[total_state_dim_ + 1]; 

  // visualize path MPC plans to take
  nav_msgs::Path path_msg;
  // path_msg.header.stamp = rclcpp::Clock::now();
  path_msg.header.frame_id = "odom";     // set the appropriate frame

  for (int i = 0; i < horizon_length_ + 1; ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    
    // Assuming state order: x, y, theta.
    pose.pose.position.x = work->solution->x[i * state_dim_ + 0];
    pose.pose.position.y = work->solution->x[i * state_dim_ + 1];
    pose.pose.position.z = 0.0;
    
    // Convert theta to a quaternion
    // double theta = work->solution->x[i * 3 + 2];
    // tf2::Quaternion q;
    // q.setRPY(0, 0, theta);
    // pose.pose.orientation = tf2::toMsg(q);
    
    path_msg.poses.push_back(pose);
  }

  path_msg_ = path_msg;

  // there is a problem with gazebo steering driver where vx doesnt move robot forward below 0.3
  std::cout<<"inital linear_vel sol: "<<linear_vel_sol_<<std::endl;
  std::cout<<"inital angular_vel sol: "<<angular_vel_sol_<<std::endl;
  if(linear_vel_sol_ < 0){
    linear_vel_sol_ = -0.29 + linear_vel_sol_;
  }
  else{
    linear_vel_sol_ = 0.29 + linear_vel_sol_;
  }

  std::cout<<"linear_vel sol: "<<linear_vel_sol_<<std::endl;
  std::cout<<"angular_vel sol: "<<angular_vel_sol_<<std::endl;

  // use x[1] to get error = xref - x[1] and use on linear_vel_sol_;
  // if(linear_vel_sol_ <= 0.3 && linear_vel_sol_ >0){
  //   linear_vel_sol_ = 0.3 + linear_vel_sol_;
  // }
  // if(linear_vel_sol_ >= -0.3 && linear_vel_sol_ < 0){
  //   linear_vel_sol_ = -0.3;
  // }

  // free osqp
  osqp_cleanup(work);
  FreeData(data);
  c_free(settings);
  return true;
}


geometry_msgs::Twist MPCController::computeVelocityCommand(Vector4d ref_state,
                                              Vector4d cur_state) {
  geometry_msgs::Twist cmd;  
  updateInitialState(cur_state);
  updateReferenceState(ref_state);
  double error_heading = normalizeAngle(ref_state(2)- cur_state(2)) ;
  heading_error_sum_ += error_heading; 
  double K = 0.0;

  if(optimize() == -1){
    ROS_ERROR("Optimization failed");
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
  }  
  ROS_INFO("Optimization done");
  cmd.linear.x = linear_vel_sol_;
  cmd.angular.z = angular_vel_sol_;

  return cmd;
}