// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

// std
#include <memory>
#include <mutex>
#include <string>
#include <vector>
// Eigen
#include <Eigen/Dense>
// ROS 
#include <ros/node_handle.h>
#include <ros/time.h>
#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>

// actionlib
#include <actionlib/server/action_server.h>
#include <moveit_msgs/DisplayTrajectory.h>

// ROS messages
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>

// realtime_tools
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_server_goal_handle.h>

// franka
#include <franka_experiments/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

namespace franka_example_controllers {

class  ShyController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time& /*time*/) override;

 private:

  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>                  ActionServer;
  typedef std::shared_ptr<ActionServer>                                                       ActionServerPtr;
  typedef ActionServer::GoalHandle                                                            GoalHandle;
  typedef trajectory_msgs::JointTrajectory::ConstPtr                                          JointTrajectoryConstPtr;
  typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction> RealtimeGoalHandle;
  typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;
  typedef realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState>     StatePublisher;
  typedef std::unique_ptr<StatePublisher>                                                     StatePublisherPtr;

  void initTrajectDeformation();
  /* \brief Reads and saves trajectory message into internal data structures */
  void parseTrajectory(const trajectory_msgs::JointTrajectory& traj);
  /* \brief Generates the trajectory deformation matrix based on the deformation length  */
  void precompute();

  /* \brief Saturation to avoid discontinuities */
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  // Trajectory action stuff 
  ActionServerPtr    action_server_;
  RealtimeGoalHandlePtr     rt_active_goal_;     ///< Currently active action goal, if any.
  /* Dynamic reconfigure CB */
  void complianceParamCallback(franka_experiments::compliance_paramConfig& config,
                               uint32_t level);
  /* Trajectory message CB*/
  void trajectoryCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);
  /* Trajectory action CB */
  virtual void goalCB(GoalHandle gh);
  virtual void cancelCB(GoalHandle gh);
  /* Cancel the active goal */
  virtual void preemptActiveGoal();
  /* Form and send action feedback TODO */
  void setActionFeedback();

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  int loop_sample_time = 1000000;      // nsec
  
  // Traject stuff
  bool haveTrajectory = false; 
  std::string robot_model_ = "panda";
  int trajectory_sample_time = 0;       // delta, nsecs
  int num_of_joints = 7;
  int trajectory_length = 0;            // samples  
  int fast_index = -1;                  // index of the fast update loop
  int slow_index = -1;                  // index of the slow (trajectory waypoint) update loop
  // trajectory message
  trajectory_msgs::JointTrajectory trajectory_;
  JointTrajectoryConstPtr trajectory_ptr_;
  // trajectory eigen data
  Eigen::MatrixXd trajectory_positions;
  Eigen::MatrixXd trajectory_velocities;
  Eigen::MatrixXi trajectory_times;
  // deformed trajectory eigen data
  Eigen::MatrixXd trajectory_frame_positions;
  Eigen::MatrixXd trajectory_deformation_;
  // trajectory deformation matrixes
  Eigen::MatrixXd A;                    // minimum jerk trajectory model matrix
  Eigen::MatrixXd R;
  Eigen::MatrixXd B;                    // waypoint paramtrization matrix
  Eigen::MatrixXd G;                    // trajectory deformation matrix        
  Eigen::MatrixXd H;                    // Shape of optimal variation
  Eigen::MatrixXd unit;                 // unit vector Nx1
  Eigen::MatrixXd Uh;                   // zero Nx1 for u
  // Desired state
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
  int time_scaling_factor = 1;             // HACK to reduce velocity
  double coriolis_factor_{1.0};

  double filter_params_{0.005};
  const double delta_tau_max_{1.0};

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_experiments::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  
  ros::Subscriber sub_trajectory_;
  ros::Subscriber trajectory_command_sub_;

};

}  // namespace franka_example_controllers
