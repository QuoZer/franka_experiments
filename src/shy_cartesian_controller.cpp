// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_experiments/shy_cartesian_controller.h>

#include <cmath>
#include <memory>
#include <thread>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_experiments/pseudo_inversion.h>

// The other method works better
//#define ALT_METHOD

namespace franka_example_controllers {

bool  ShyCartesianController::init(hardware_interface::RobotHW* robot_hw,
                                      ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  // save nh
  controller_nh_ = node_handle;

  sub_trajectory_ = node_handle.subscribe(
      "move_group/display_planned_path", 20, & ShyCartesianController::trajectoryCallback, this,   // TODO: change topic name instead of remapping
      ros::TransportHints().reliable().tcpNoDelay());

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM(" ShyCartesianController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        " ShyCartesianController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  if (!node_handle.getParam("trajectory_deformed_length", deformed_segment_length)) {
    ROS_ERROR_STREAM(" ShyCartesianController: Could not read parameter trajectory_deformed_length");
    return false;
  }
  
  if (!node_handle.getParam("admittance", admittance)) {
    ROS_ERROR_STREAM(" ShyCartesianController: Could not read parameter admittance");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        " ShyCartesianController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        " ShyCartesianController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        " ShyCartesianController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        " ShyCartesianController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }
  
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        " ShyCartesianController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          " ShyCartesianController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // std::vector<double> force_thresholds;
  // if (!node_handle.getParam("force_non_sensitivity_threshold", force_thresholds)) {
  //   ROS_ERROR(
  //       "ShyController:  Invalid or no force_thresholds provided, aborting "
  //       "controller init!");
  //   return false;
  // }
  // force_thresholds_vector = Eigen::Matrix<double, 6, 1>(force_thresholds.data());

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_experiments::cart_compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(& ShyCartesianController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  // ROS API: Subscribed topics
  trajectory_command_sub_ = node_handle.subscribe("command", 1, &ShyCartesianController::trajectoryCallback, this);

  // ROS API: Published topics
  marker_publisher_.reset(new MarkerPublisher(node_handle, "MarkerArray", 1));

  // ROS API: Action interface
  action_server_.reset(
      new ActionServer(node_handle, "follow_joint_trajectory",
                       std::bind(&ShyCartesianController::goalCB, this, std::placeholders::_1),
                       std::bind(&ShyCartesianController::cancelCB, this, std::placeholders::_1), false));
  action_server_->start();

  return true;
}

void  ShyCartesianController::starting(const ros::Time& /*time*/) {
  franka::RobotState initial_state = state_handle_->getRobotState();
  
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
  
  have_trajectory = false; // to be sure
  fast_index = -1;
  slow_index = -1;
  deformed_segment_length = std::max(10, static_cast<int>(std::floor(trajectory_length*deformed_segment_ratio_target_)));
  precompute(deformed_segment_length);

  ROS_INFO("ShyCartesianController: Starting controller");
}

void ShyCartesianController::precompute(int N)
{
  // Deformations precompute
  //int N = trajectory_deformed_length;
  unit = Eigen::MatrixXd::Ones(N, 1);
  Uh = Eigen::MatrixXd::Zero(N, 7);
  pos_segment_deformation = Eigen::MatrixXd::Zero(N, 3);
  ori_segment_deformation = Eigen::MatrixXd::Zero(N, 4);
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(N, N);

  // method from "Trajectory Deformations from Physical Human-Robot Interaction"
  // minimum jerk trajectory model matrix
  A = Eigen::MatrixXd::Zero(N + 3, N);
  // Fill diagonal with 1s
  A.diagonal(0).setConstant(1);
  // Fill the diagonal below it with -3
  A.diagonal(-1).setConstant(-3);
  // Fill the diagonal below -3 with 3
  A.diagonal(-2).setConstant(3);
  // Fill the diagonal below 3 with -1
  A.diagonal(-3).setConstant(-1);
  
  R = A.transpose() * A;
  // Ensures that the first two and the last two points are the same as in the original trajectory
  B = Eigen::MatrixXd::Zero(4, N);
  B(0, 0) = 1;
  B(1, 1) = 1;
  B(2, N - 2) = 1;
  B(3, N - 1) = 1;

  G = (I - R.inverse() * B.transpose() * (B * R.inverse() * B.transpose()).inverse() * B ) * R.inverse() * unit ;    

  H_full = std::sqrt(N) * G / G.norm();  
  H = H_full;  

  // ROS_INFO("Finished precompute");
}

void  ShyCartesianController::update(const ros::Time& time,
                            const ros::Duration& period) {
  // Update time data (this block is taken from the OG joint traj conroller)
  prev_time_data_ = *(time_data_.readFromRT());
  TimeData time_data;
  time_data.time   = time;                                     // Cache current time
  time_data.period = period;                                   // Cache current control period
  time_data.uptime = prev_time_data_.uptime + period;          // Update controller uptime
  time_data_.writeFromNonRT(time_data);                        // TODO: Grrr, we need a lock-free data structure here!

  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  // convert to Eigen
  robot_mode = robot_state.robot_mode; 

  // franka RobotState can be used instead, but to keep thing uniformal in feedback pub...
  State desired_state;  
  State current_state;
  current_state.position = std::vector<double>(robot_state.q.data(), robot_state.q.data() + robot_state.q.size());
  current_state.velocity = std::vector<double>(robot_state.dq.data(), robot_state.dq.data() + robot_state.dq.size());
  current_state.time_from_start = ros::Duration(time_data.uptime.toSec(), time_data.uptime.toNSec());

  // update parameters changed online through dynamic reconfigure 
  std::lock_guard<std::mutex> lock(admittance_mutex_);
  admittance = admittance_target_;
  if (deformed_segment_ratio != deformed_segment_ratio_target_) need_recompute = true;
  deformed_segment_ratio = deformed_segment_ratio_target_;

  // TRAJECTORY DEFORMATION
  if (have_trajectory) {
    fast_index++;
    trajectory_sample_time = trajectory_times(slow_index+1, 0);
  }
  if (have_trajectory && fast_index*loop_sample_time >= trajectory_sample_time)    //  TODO: switch to time-based (instead of index-based) trajectory sampling?
  {
    slow_index++;
    fast_index = 0;

    if (pos_segment_deformation.rows() !=  H.rows() ) {
      ROS_ERROR("ShyCartesianController: segment_deformation.rows() !=  H.rows()");
    }

    getDeformedGoal(robot_state, position_d_, orientation_d_);

    // termination, resetting goal
    if (slow_index == trajectory_length - 1)
    {
      successActiveGoal();
    }
    // TODO: fix and return feedback publishing

    publishTrajectoryMarkers(trajectory_positions, 1);
  } // end traj deform
  else if (need_recompute)  // updating deformation matrix only in timesteps when no deformation takes place
  {
    deformed_segment_length = static_cast<int>(std::max(10, static_cast<int>(std::floor(trajectory_length*deformed_segment_ratio)))); 
    downsampleDeformation(deformed_segment_length);
    need_recompute = false;
  }

  Eigen::VectorXd tau_d(7);
  computeTau(robot_state, tau_d);

  // sending tau to the robot
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  // deformed_segment_length = std::max(10, static_cast<int>(std::floor(trajectory_length*deformed_segment_ratio_target_)));
}  // end update()

void ShyCartesianController::getDeformedGoal(franka::RobotState& robot_state,  
                       Eigen::Vector3d& position_d_, 
                       Eigen::Quaterniond& orientation_d_)
{
  Eigen::Map<Eigen::Matrix<double, 6, 1>> fh(robot_state.K_F_ext_hat_K.data());
  // Subtracting the force threshold from the measured force to correct for the calm state force
  //fh -= force_thresholds_vector;
  // Filtering the noise from the force measurement
  //fh = ((fh.array() > -0.1) && (fh.array() < 0.1)).select(0, fh);

  //  Nx3 = 1x1 * Nx1 * 1x3   (deforming only position for now)
  pos_segment_deformation = admittance * trajectory_sample_time/pow(10, 9) * H * fh.block(0, 0, 3, 1).transpose();

  int remaining_size = trajectory_deformation.rows() - slow_index;
  int short_vector_effective_size = std::min((int)pos_segment_deformation.rows(), remaining_size);
  // Additions to the map object are reflected in the original matrix
  trajectory_deformation.block(slow_index, 0, short_vector_effective_size, 3) += pos_segment_deformation.topRows(short_vector_effective_size);
  
  // Update targets
  position_d_ = trajectory_deformation.row(slow_index) + trajectory_positions.row(slow_index);
  orientation_d_ = Eigen::Quaterniond(trajectory_orientations(slow_index, 3),
                                      trajectory_orientations(slow_index, 0), 
                                      trajectory_orientations(slow_index, 1), 
                                      trajectory_orientations(slow_index, 2));
}

void ShyCartesianController::computeTau(franka::RobotState& robot_state, Eigen::VectorXd& tau_d)
{
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());  
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  ros::Time uptime = time_data_.readFromRT()->uptime;
  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;
  if (have_trajectory && (error.head(3).maxCoeff() > 0.1 || error.head(3).minCoeff() < -0.1)) 
  {
    ROS_WARN("Position error is too big. Dropping the goal");
    preemptActiveGoal();
    this->startRequest(uptime);
  }

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);

  if (!error.allFinite()) {
    ROS_ERROR("Error is not finite. Stopping the controller.");
    preemptActiveGoal();
    this->stopRequest(uptime);
  }
  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7);
  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                    (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                    (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);  
}

void ShyCartesianController::stopping(const ros::Time& /*time*/)
{
  preemptActiveGoal();
}

void ShyCartesianController::successActiveGoal()
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
  ROS_INFO("Trajectory execution finished after %d waypoints", slow_index+1);
  slow_index = 0;
  have_trajectory = false;
  if (current_active_goal) {
    current_active_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    current_active_goal->setSucceeded(current_active_goal->preallocated_result_);        // TODO: why tf it doesn't work?
    current_active_goal->runNonRealtime(ros::TimerEvent());                              // now it works. doesn't look realtime-safe though
    current_active_goal.reset(); 
    rt_active_goal_.reset();
  }
}

void ShyCartesianController::setActionFeedback(State& desired_state, State& current_state)
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
  if (!current_active_goal)
  {
    return;
  }
  
  current_active_goal->preallocated_feedback_->header.stamp            = time_data_.readFromRT()->time;
  current_active_goal->preallocated_feedback_->desired.positions       = desired_state.position;
  current_active_goal->preallocated_feedback_->desired.velocities      = desired_state.velocity;
  current_active_goal->preallocated_feedback_->desired.accelerations   = desired_state.acceleration;
  current_active_goal->preallocated_feedback_->desired.time_from_start = desired_state.time_from_start;
  current_active_goal->preallocated_feedback_->actual.positions        = current_state.position;
  current_active_goal->preallocated_feedback_->actual.velocities       = current_state.velocity;
  current_active_goal->preallocated_feedback_->actual.time_from_start  = current_state.time_from_start;
  // current_active_goal->preallocated_feedback_->error.positions       = state_error_.position;
  // current_active_goal->preallocated_feedback_->error.velocities      = state_error_.velocity;
  // current_active_goal->preallocated_feedback_->error.time_from_start = ros::Duration(state_error_.time_from_start);
  current_active_goal->setFeedback( current_active_goal->preallocated_feedback_ );

}


Eigen::Matrix<double, 7, 1>  ShyCartesianController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

// Callback functions are below 

void  ShyCartesianController::complianceParamCallback(
    franka_experiments::cart_compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
  std::lock_guard<std::mutex> lock(admittance_mutex_);
  admittance_target_ = config.admittance;
  deformed_segment_ratio_target_ = config.deformed_length;
}

void ShyCartesianController::parseTrajectory(const trajectory_msgs::JointTrajectory& traj)
{
  num_of_joints = traj.joint_names.size();
  trajectory_length = traj.points.size();
  // Convert to eigen
  trajectory_positions    = Eigen::MatrixXd(trajectory_length, 3);
  trajectory_orientations = Eigen::MatrixXd(trajectory_length, 4);
  trajectory_deformation  = Eigen::MatrixXd::Zero(trajectory_length, 7);
  trajectory_times        = Eigen::MatrixXi(trajectory_length, 1); 
  
  // update from dynamic reconfigure
  deformed_segment_length = static_cast<int>(std::floor(trajectory_length*deformed_segment_ratio));
  deformed_segment_length = std::max(10, deformed_segment_length);    // we need some points anyway
  precompute(trajectory_length);    // just set the flag and let the update() do the job ???
  downsampleDeformation(deformed_segment_length);
  
  // probably can be done in a more efficient way
  int prev_ts = 0;
  Eigen::Vector3d translation;
  Eigen::Vector4d orientation;
  for (int i = 0; i < trajectory_length; i++){
    forwardKinematics(traj.points[i].positions, translation, orientation);
    trajectory_positions.row(i) = translation.transpose();
    trajectory_orientations.row(i) = orientation.transpose();
    // filling with time differences
    trajectory_times(i, 0) = time_scaling_factor*(traj.points[i].time_from_start.toNSec() - prev_ts);
    prev_ts = traj.points[i].time_from_start.toNSec();
  }

  fillFullTrajectoryMarkers(trajectory_positions, 4);

  ROS_INFO("Received a new trajectory with %d waypoints. Deformation frame length %d, current admittance %f",
                 trajectory_length, deformed_segment_length, admittance);
}

void ShyCartesianController::downsampleDeformation(int new_N)
{
    // Check if N is greater than original size, return 
    if (new_N > trajectory_length) {
      return ;
    }

    H = Eigen::MatrixXd(new_N, 1);
    pos_segment_deformation = Eigen::MatrixXd::Zero(new_N, 3);
    ori_segment_deformation = Eigen::MatrixXd::Zero(new_N, 4);

    double stride = static_cast<double>(H_full.size() - 1) / (new_N - 1);

    for (int i = 0; i < new_N; ++i) {
        int index = static_cast<int>(std::round(i * stride));
        H(i) = H_full(index);
    }

}

void  ShyCartesianController::trajectoryCallback(
    const moveit_msgs::DisplayTrajectory::ConstPtr& msg) {

  if (have_trajectory){
    ROS_WARN("Received a new trajectory message while the old one is still being executed. Ignoring the new trajectory");    
    return;
  }

  if (!msg->model_id.empty() && msg->model_id != robot_model_)
    ROS_WARN("Received a trajectory to display for model '%s' but model '%s' was expected", msg->model_id.c_str(), robot_model_.c_str());

  trajectory_ = msg->trajectory[0].joint_trajectory;
  parseTrajectory(trajectory_);    
  
  have_trajectory = true;
  preemptActiveGoal();
} 

void ShyCartesianController::goalCB(GoalHandle gh)
{
  // Preconditions:
  if (robot_mode == franka::RobotMode::kUserStopped || robot_mode == franka::RobotMode::kReflex)
  {
    ROS_WARN("Can't accept new action goals. Check the user stop button.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string = "Robot is not ready";
    gh.setRejected(result);
    return;   
  }
  if (!this->isRunning())
  {
    ROS_WARN("Can't accept new action goals. Controller is not running.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string = "Controller is not running";
    gh.setRejected(result);
    return;
  }
  if (have_trajectory)
  {
    ROS_WARN("Received a new trajectory action while the old one is still being executed. Ignoring the new trajectory");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string = "Prev trajectory is still being executed";
    gh.setRejected(result);
    return;
  }

  // Try to update new trajectory
  RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
  trajectory_ = gh.getGoal()->trajectory;

  parseTrajectory(trajectory_);

  preemptActiveGoal();
  
  gh.setAccepted();
  rt_active_goal_ = rt_goal;
  have_trajectory = true;

  // Setup goal status checking timer
  goal_handle_timer_ = controller_nh_.createTimer(ros::Duration(1.0 / action_monitor_rate),
                                                  &RealtimeGoalHandle::runNonRealtime,
                                                  rt_goal);
  goal_handle_timer_.start();
}

void ShyCartesianController::cancelCB(GoalHandle gh)
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

  // Check that cancel request refers to currently active goal (if any)
  if (current_active_goal && current_active_goal->gh_ == gh)
  {
    // Reset current goal
    rt_active_goal_.reset();
    ROS_INFO("Canceling active action goal because cancel callback recieved from actionlib.");

    // Mark the current goal as canceled
    current_active_goal->gh_.setCanceled();
    this->startRequest(gh.getGoal()->trajectory.header.stamp);    // not sure about the argument
  }
}

inline void ShyCartesianController::preemptActiveGoal()
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

  // Cancels the currently active goal
  if (current_active_goal)
  {
    // Marks the current goal as canceled
    rt_active_goal_.reset();
    current_active_goal->gh_.setCanceled();
  }
}

// Visualization related things below 

void ShyCartesianController::publishTrajectoryMarkers(Eigen::MatrixXd& trajectory, int frequency)
{
  if (marker_publisher_ && marker_publisher_->trylock())
  {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "panda_link0";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0;
    marker.color.b = 0.0;

    Eigen::Vector3d translation;
    Eigen::Vector4d orientation;
    Eigen::MatrixXd q_def = Eigen::MatrixXd::Zero(1, 7);
    int target_index = (int)std::min(slow_index + deformed_segment_length, trajectory_length - 1) / frequency;
    for (int i = 0; i < trajectory.rows(); i+=frequency)
    {
      //Sq_def = trajectory.row(i) - trajectory_deformation.row(i);
      translation = trajectory_positions.row(i) + trajectory_deformation.row(i);
      //forwardKinematics(q_def.transpose(), translation, orientation);
      marker.id = i;
      marker.pose.position.x = translation(0);
      marker.pose.position.y = translation(1);
      marker.pose.position.z = translation(2);
      if (i == target_index) {   // deformation horizon
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
      }
      else {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
      }

      markers.markers.push_back(marker);
    }
    marker.id = trajectory.rows();
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    std::ostringstream oss;
    oss << "Admittance: " << admittance << " \nDeformation length: " << deformed_segment_length;
    marker.text = oss.str();
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 1;
    marker.scale.z = 0.1;
    markers.markers.push_back(marker);
    // Append full trajectory markers
    markers.markers.insert(markers.markers.end(), full_trajectory_markers_.markers.begin(), full_trajectory_markers_.markers.end());
    if (slow_index == trajectory_length) markers.markers.clear(); // clear markers if trajectory is finished
    marker_publisher_->msg_ = markers;
    marker_publisher_->unlockAndPublish();

  }
}

void ShyCartesianController::fillFullTrajectoryMarkers(Eigen::MatrixXd& trajectory, int frequency)
{
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "panda_link0";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  marker.color.a = 0.7;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  Eigen::Vector3d translation;
  Eigen::Vector4d orientation;
  for (int i = 0; i < trajectory.rows(); i+=frequency)
  {
    forwardKinematics(trajectory_.points[i].positions, translation, orientation);
    marker.id = i;
    marker.pose.position.x = translation(0);
    marker.pose.position.y = translation(1);
    marker.pose.position.z = translation(2);
    markers.markers.push_back(marker);
  }
  full_trajectory_markers_.markers.clear();
  full_trajectory_markers_ = markers;
}

Eigen::Matrix<double, 8, 4> ShyCartesianController::dh_params(const std::vector<double>& joint_variable) 
{
  // Create DH parameters (data given by maker franka-emika)
  dh <<   0,      0,        0.333,   joint_variable[0],
       -M_PI/2,   0,        0,       joint_variable[1],
        M_PI/2,   0,        0.316,   joint_variable[2],
        M_PI/2,   0.0825,   0,       joint_variable[3],
       -M_PI/2,  -0.0825,   0.384,   joint_variable[4],
        M_PI/2,   0,        0,       joint_variable[5],
        M_PI/2,   0.088,    0.107,   joint_variable[6],
        0,        0,        0.103,   -M_PI/4;

  return dh;
}

Eigen::Matrix4d ShyCartesianController::TF_matrix(int i, const Eigen::Matrix<double, 8, 4>& dh) 
{
  double alpha = dh(i, 0);
  double a = dh(i, 1);
  double d = dh(i, 2);
  double q = dh(i, 3);

  Eigen::Matrix4d TF;
  TF << cos(q),             -sin(q),              0,            a,
        sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d,
        sin(q) * sin(alpha), cos(q) * sin(alpha),  cos(alpha),  cos(alpha) * d,
        0, 0, 0, 1;
  return TF;
}

void ShyCartesianController::forwardKinematics(const std::vector<double>& joint_pose, Eigen::Vector3d& translation, Eigen::Vector4d& orientation)
{
  Eigen::Matrix<double, 8, 4> dh_parameters = dh_params(joint_pose);

  Eigen::Matrix4d T_01 = TF_matrix(0, dh_parameters);
  Eigen::Matrix4d T_12 = TF_matrix(1, dh_parameters);
  Eigen::Matrix4d T_23 = TF_matrix(2, dh_parameters);
  Eigen::Matrix4d T_34 = TF_matrix(3, dh_parameters);
  Eigen::Matrix4d T_45 = TF_matrix(4, dh_parameters);
  Eigen::Matrix4d T_56 = TF_matrix(5, dh_parameters);
  Eigen::Matrix4d T_67 = TF_matrix(6, dh_parameters);
  Eigen::Matrix4d T_78 = TF_matrix(7, dh_parameters);

  Eigen::Matrix4d T_08 = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * T_67 * T_78;
  tf::Matrix3x3 tf3d(T_08(0, 0), T_08(0, 1), T_08(0, 2),
                     T_08(1, 0), T_08(1, 1), T_08(1, 2),
                     T_08(2, 0), T_08(2, 1), T_08(2, 2));
  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);
  translation = Eigen::Block<Eigen::Matrix4d, 3, 1>(T_08, 0, 3);
  orientation = Eigen::Vector4d(tfqt.x(), tfqt.y(), tfqt.z(), tfqt.w());
}


}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ShyCartesianController,   //
                       controller_interface::ControllerBase)
