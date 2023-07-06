// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <moveit_msgs/DisplayTrajectory.h>

#include <franka_experiments/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_example_controllers {

class  ShyController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  void initTrajectDeformation();
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  int loop_sample_time = 1000000;      // nsec
  
  // Traject stuff
  bool haveTrajectory = false; 
  std::string robot_model_ = "panda";
  // trajectory message
  trajectory_msgs::JointTrajectory trajectory_;
  trajectory_msgs::JointTrajectory trajectory_deformed_; // rm
  // trajectory eigen data
  Eigen::MatrixXd trajectory_positions;
  Eigen::MatrixXd trajectory_velocities;
  Eigen::MatrixXi trajectory_times;
  // deformed trajectory eigen data
  Eigen::MatrixXd trajectory_frame_positions;
  Eigen::MatrixXd trajectory_deformation_;
  int trajectory_sample_time = 0;       // delta, nsecs
  int num_of_joints = 7;
  int trajectory_length = 0;            // samples  
  Eigen::MatrixXd A;                    // minimum jerk trajectory model matrix
  Eigen::MatrixXd R;
  Eigen::MatrixXd B;                    // waypoint paramtrization matrix
  Eigen::MatrixXd G;                    // trajectory deformation matrix        
  Eigen::MatrixXd H;                    // Shape of optimal variation
  Eigen::MatrixXd unit;                 // unit vector Nx1
  Eigen::MatrixXd Uh;              // zero Nx1 for u
  int fast_index = -1;
  int slow_index = -1;
  Eigen::Matrix<double, 7, 1> dq_filtered_;
  Eigen::Matrix<double, 7, 1> q_d, delta_q, dq_d;      // desired joint position and velocity  
  //Eigen::Matrix<double, 7, 1> prev_q_d, prev_dq_d;
  Eigen::Matrix<double, 7, 1> tau_d_calculated;
  Eigen::Matrix<double, 7, 1> tau_d_saturated;
  // PARAMETERS`
  Eigen::MatrixXd k_gains_;
  Eigen::MatrixXd d_gains_;
  double admittance = 0;                   // nu  
  double admittance_target_ = 0;            // nu for dynamic reconf
  std::mutex admittance_mutex_;
  int trajectory_deformed_length = 1;      // N, samples
  int trajectory_deformed_length_target_ = 1; // N for dynamic reconf
  double coriolis_factor_{1.0};

  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_experiments::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(franka_experiments::compliance_paramConfig& config,
                               uint32_t level);

  // Equilibrium pose subscriber
//   ros::Subscriber sub_equilibrium_pose_;
  ros::Subscriber sub_trajectory_;
//   void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void trajectoryCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);
};

}  // namespace franka_example_controllers
