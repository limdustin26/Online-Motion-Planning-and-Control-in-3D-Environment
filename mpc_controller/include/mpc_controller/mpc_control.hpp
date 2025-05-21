#ifndef _CONTROLLER_MPC_CONTROL_H
#define _CONTROLLER_MPC_CONTROL_H

#include "ros/ros.h"
#include "Eigen/Eigen"
#include "Eigen/Sparse"
#include "btraj_msgs/trajectory.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Twist.h"
#include <vector>
#include <osqp/osqp.h>
using namespace Eigen;
using namespace std;

class MPCController
{
private:

    // MPC
    // linear equality constraint (motion model)
    // MatrixXd A_, B_;
    SparseMatrix<double> C_;
    SparseMatrix<double> P_;
    std::vector<c_float> q_;
    std::vector<c_float> l_;
    std::vector<c_float> u_;

    nav_msgs::Path path_msg_;

    double sample_t_;
    int total_dim_;
    int total_state_dim_;
    int total_control_dim_;
    int horizon_length_;
    int state_dim_; // system state dimension
    int control_dim_; // control input dimension
    double longitudinal_state_cost_;
    double lateral_state_cost_;
    double orientation_state_cost_;
    double terminal_state_cost_;
    double linear_control_cost_;
    double linear_vel_max_;
    double angular_vel_max_;
    double linear_vel_sol_;
    double angular_vel_sol_;
    double heading_error_sum_;
    double angular_control_cost_;

protected:
    static void FreeData(OSQPData* data);
    template <typename T>
    // T* CopyData(const std::vector<T>& vec) {
    //     T* data = new T[vec.size()]();
    //     memcpy(data, vec.data(), sizeof(T) * vec.size());
    //     return data;
    // }
    T* CopyData(const std::vector<T>& vec) {
        size_t size = vec.size();
        T* data = new T[size];  // Allocate memory
    
        for (size_t i = 0; i < size; ++i) {
            data[i] = vec[i];  // Copy each element manually
        }
    
        return data;
    }
    

    

public:
    MPCController();  // Constructor declaration
    ~MPCController(); // Destructor declaration
    void initMPC();
    // int find_nearest_pt(vector<Vector4d>& ref  , Vector4d current_ps);
    int find_nearest_pt(vector<Vector4d> &ref, Vector4d current_ps, int current_idx);
    int find_nearest_pt(vector<Vector4d> &ref, Vector4d current_ps);
    void setparams(int horizon_length, double sample_t, double longitudinal_state_cost, double lateral_state_cost, double orientation_state_cost, double terminal_state_cost, 
        double control_cost, double angular_control_cost,double min_turning_radius, double linear_vel);
    void updateReferenceState(Vector4d ref_state);
    void updateInitialState(Vector4d cur_state);
    double normalizeAngle(double angle);
    geometry_msgs::Twist computeVelocityCommand(Vector4d ref_state, Vector4d cur_state);
    int optimize();
    csc* convertEigenToCSC(const Eigen::SparseMatrix<double>& mat);
    double compute_norm(const c_float* vec, int size);
    void print_residuals(OSQPWorkspace* work);
    inline void setErrorSumZero(){
        heading_error_sum_ = 0;
    }
    inline nav_msgs::Path getMPCPath(){
        return path_msg_;
    }

};

#endif