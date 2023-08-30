// Derived from Franka's joint trajectory controller. 
// Based on "Trajectory Deformations from Physical Human-Robot Interaction" paper
// Developed by: @QuoZer

#include <franka_experiments/shy_controller.h>

#include <cmath>
#include <memory>
#include <thread>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_experiments/pseudo_inversion.h>

namespace franka_example_controllers {

bool  ShyController::init(hardware_interface::RobotHW* robot_hw,
                                      ros::NodeHandle& node_handle) {
  // save nh
  controller_nh_ = node_handle;

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM(" ShyController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        " ShyController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  if (!node_handle.getParam("trajectory_deformed_length", deformed_segment_length)) {
    ROS_ERROR_STREAM(" ShyController: Could not read parameter trajectory_deformed_length");
    return false;
  }
  
  if (!node_handle.getParam("admittance", admittance)) {
    ROS_ERROR_STREAM(" ShyController: Could not read parameter admittance");
    return false;
  }

  std::vector<double> k_gains_vec;
  if (!node_handle.getParam("k_gains", k_gains_vec) || k_gains_vec.size() != 7) {
    ROS_ERROR(
        "ShyController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }
  k_gains_ =  Eigen::Map<Eigen::Matrix<double, 7, 1>>(k_gains_vec.data()).asDiagonal();

  std::vector<double> d_gains_vec;
  if (!node_handle.getParam("d_gains", d_gains_vec) || d_gains_vec.size() != 7) {
    ROS_ERROR(
        "ShyController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }  
  d_gains_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(d_gains_vec.data()).asDiagonal();

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        " ShyController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        " ShyController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        " ShyController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        " ShyController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }
  
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        " ShyController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          " ShyController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }


  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_experiments::compliance_paramConfig>>(
      dynamic_reconfigure_compliance_param_node_);

  dynamic_server_compliance_param_->setCallback(
      boost::bind(& ShyController::complianceParamCallback, this, _1, _2));

  // ROS API: Subscribed topics
  //trajectory_command_sub_ = node_handle.subscribe("command", 1, &ShyController::trajectoryCallback, this);
  sub_trajectory_ = node_handle.subscribe(
      "move_group/display_planned_path", 20, & ShyController::trajectoryCallback, this,   // TODO: change topic name instead of remapping
      ros::TransportHints().reliable().tcpNoDelay());

  // ROS API: Published topics
  marker_publisher_.reset(new MarkerPublisher(node_handle, "MarkerArray", 1));

  // ROS API: Action interface
  action_server_.reset(
      new ActionServer(node_handle, "follow_joint_trajectory",
                       std::bind(&ShyController::goalCB, this, std::placeholders::_1),
                       std::bind(&ShyController::cancelCB, this, std::placeholders::_1), false));
  action_server_->start();

  return true;
}

void  ShyController::starting(const ros::Time& /*time*/) {
  franka::RobotState initial_state = state_handle_->getRobotState();
  
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data()); //

  // set target positions to initial position
  q_d = q_initial;
  dq_initial.fill(0);
  dq_d = dq_initial;   
  delta_q = dq_initial;
  
  have_trajectory = false; // to be sure
  fast_index = -1;
  slow_index = -1;
  deformed_segment_length = std::max(10, static_cast<int>(std::floor(trajectory_length*deformed_segment_ratio_target_)));
  precompute(deformed_segment_length);

  ROS_INFO("ShyController: Starting controller");
}

void ShyController::precompute(int N)
{
  // Deformations precompute
  unit = Eigen::MatrixXd::Ones(N, 1);
  Uh = Eigen::MatrixXd::Zero(N, 7);
  segment_deformation     = Eigen::MatrixXd::Zero(N, num_of_joints);
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

  ROS_INFO("ShyController: Finished precompute");
}

void  ShyController::update(const ros::Time& time,
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
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
  robot_mode = robot_state.robot_mode; 

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
    // apply trajectory deformation and update q_d and dq_d
    getDeformedGoal(robot_state, q_d, dq_d);
    
    // termination, resetting goal
    if (slow_index == trajectory_length - 1)    // closeness check?
    {
      successActiveGoal();
    }

    setActionFeedback(time_data, robot_state, q_d, dq_d); 
    publishTrajectoryMarkers(trajectory_positions);       // TODO: decouple visualization from deformation loop 
  } // end traj deform
  else if (need_recompute)  // updating the deformation matrix only in timesteps when no deformation takes place for perfomance
  {
    deformed_segment_length = static_cast<int>(std::floor(trajectory_length*deformed_segment_ratio)); 
    downsampleDeformation(deformed_segment_length);
    need_recompute = false;
  }
  
  // sanity check
  if (!(trajectory_positions.allFinite() && q_d.allFinite() && dq_d.allFinite()))
  {
    throw std::runtime_error("Trajectory positions, q_d or dq_d are not finite");
  }
  if ( ( (q_d-q).maxCoeff() > 0.1 || (q_d-q).minCoeff() < -0.1) && have_trajectory) // probably the condition is a bit too basic. 
  {
    preemptActiveGoal();
    this->startRequest(time_data.uptime);
    ROS_WARN("Trajectory positions are too far from current robot state. Dropping the goal");
    //throw std::runtime_error("Trajectory positions are too far from current robot state");
  }

  // filtering from the joint impedance example
  double alpha = 0.99;
  dq_filtered_ = (1 - alpha) * dq_filtered_ + alpha * dq;
  // impedance control
  tau_d_calculated = coriolis_factor_ * coriolis +
                            k_gains_ * (q_d - q) +
                            d_gains_ * (dq_d - dq_filtered_);
      
  // saturation to avoid discontinuities
  tau_d_saturated = saturateTorqueRate(tau_d_calculated, tau_J_d);
  // sending tau to the robot
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }

}  // end update()

void ShyController::stopping(const ros::Time& /*time*/)
{
  preemptActiveGoal();
}

void ShyController::getDeformedGoal(franka::RobotState& robot_state,  
                                    Eigen::Matrix<double, 7, 1>& q_d, 
                                    Eigen::Matrix<double, 7, 1>& dq_d)
{
  Eigen::Map<Eigen::Matrix<double, 7, 1>> uh(robot_state.tau_ext_hat_filtered.data());

  if (segment_deformation.rows() !=  H.rows() ) {
    ROS_ERROR("ShyController: segment_deformation.rows() !=  H.rows()");
  }

  // Nx7 = 1x1 * Nx1 * 1x7
  segment_deformation = admittance * trajectory_sample_time/pow(10, 9) * H * uh.transpose();
  // Eigen::Matrix<double, 7,7> k_inv = k_gains_.inverse();
  // could set actual admittance to reverse stiffness 
  //segment_deformation = admittance * trajectory_sample_time/pow(10, 9) * H * uh.transpose() * k_inv;

  int remaining_size = trajectory_deformation.rows() - slow_index;
  int short_vector_effective_size = std::min((int)segment_deformation.rows(), remaining_size);
  // Additions to the map object are reflected in the original matrix
  trajectory_deformation.block(slow_index, 0, short_vector_effective_size, 7) += segment_deformation.topRows(short_vector_effective_size);

  // update q_d and qd_d
  q_d = trajectory_positions.row(slow_index) - trajectory_deformation.row(slow_index);
  delta_q = trajectory_positions.row(slow_index+1) - trajectory_deformation.row(slow_index+1) - q_d.transpose().row(0);    // ugly and probably dangerous
  if (slow_index == 0 || slow_index == trajectory_length - 1) 
    dq_d = Eigen::MatrixXd::Zero(7, 1).row(0);    // zero velocity at the start and end
  else 
    dq_d = delta_q * pow(10, 9) / trajectory_sample_time;   //nsec to sec
}

void ShyController::setActionFeedback(const TimeData& time_data, 
                                      const franka::RobotState& robot_state, 
                                      Eigen::Matrix<double, 7, 1> q_d, 
                                      Eigen::Matrix<double, 7, 1> dq_d)
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
  if (!current_active_goal)
  {
    return;
  }

  control_msgs::FollowJointTrajectoryFeedbackPtr feedback = current_active_goal->preallocated_feedback_;

  feedback->header.stamp = time_data_.readFromRT()->time;
  feedback->desired.positions       = std::vector<double>(q_d.data(), q_d.data() + q_d.size());
  feedback->desired.velocities      =  std::vector<double>(dq_d.data(), dq_d.data() + dq_d.size());
  feedback->desired.accelerations   =  std::vector<double>(7, 0.0);
  feedback->desired.time_from_start =  ros::Duration(time_data.uptime.toSec(), time_data.uptime.toNSec());
  feedback->actual.positions        =  std::vector<double>(robot_state.q.data(), robot_state.q.data() + robot_state.q.size());
  feedback->actual.velocities       =  std::vector<double>(robot_state.dq.data(), robot_state.dq.data() + robot_state.dq.size());
  feedback->actual.time_from_start  =  ros::Duration(time_data.uptime.toSec(), time_data.uptime.toNSec());
  // current_active_goal->preallocated_feedback_->error.positions       = state_error_.position;
  // current_active_goal->preallocated_feedback_->error.velocities      = state_error_.velocity;
  // current_active_goal->preallocated_feedback_->error.time_from_start = ros::Duration(state_error_.time_from_start);
  current_active_goal->setFeedback( current_active_goal->preallocated_feedback_ );

}

Eigen::Matrix<double, 7, 1>  ShyController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) 
{  
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

// Callback functions below 

void  ShyController::complianceParamCallback(
    franka_experiments::compliance_paramConfig& config,
    uint32_t /*level*/) {
  std::lock_guard<std::mutex> lock(admittance_mutex_);
  admittance_target_ = config.admittance;
  deformed_segment_ratio_target_ = config.deformed_length;
}

void ShyController::parseTrajectory(const trajectory_msgs::JointTrajectory& traj)
{
  num_of_joints = traj.joint_names.size();
  trajectory_length = traj.points.size();
  // Convert to eigen
  trajectory_positions    = Eigen::MatrixXd(trajectory_length, num_of_joints);
  trajectory_deformation  = Eigen::MatrixXd::Zero(trajectory_length, num_of_joints);
  trajectory_times        = Eigen::MatrixXi(trajectory_length, 1); 
  
  // update from dynamic reconfigure
  deformed_segment_length = static_cast<int>(std::floor(trajectory_length*deformed_segment_ratio));
  precompute(trajectory_length);    // just set the flag and let the update() do the job ???
  downsampleDeformation(deformed_segment_length);
  
  // probably can be done in a more efficient way
  int prev_ts = 0;
  for (int i = 0; i < trajectory_length; i++){
    for (int j = 0; j < num_of_joints; j++){
      trajectory_positions(i, j) = traj.points[i].positions[j];
    }
    // filling  time differences
    trajectory_times(i, 0) = time_scaling_factor*(traj.points[i].time_from_start.toNSec() - prev_ts);
    prev_ts = traj.points[i].time_from_start.toNSec();
  }

  fillFullTrajectoryMarkers(trajectory_positions, 4);

  ROS_INFO("Received a new trajectory with %d waypoints. Deformation frame length %d, current admittance %f",
                 trajectory_length, deformed_segment_length, admittance);
}

void ShyController::downsampleDeformation(int new_N)
{
  // To have at least some points 
  new_N = static_cast<int>(std::max(10, new_N)); 
  // Check if N is greater than original size, return (idk how this can happen though)
  if (new_N > trajectory_length) {
    return ;
  }

  H = Eigen::MatrixXd(new_N, 1);
  segment_deformation = Eigen::MatrixXd::Zero(new_N, num_of_joints);
  
  double stride = static_cast<double>(H_full.size() - 1) / (new_N - 1);

  for (int i = 0; i < new_N; ++i) {
      int index = static_cast<int>(std::round(i * stride));
      H(i) = H_full(index);
  }

}

void  ShyController::trajectoryCallback(
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

void ShyController::goalCB(GoalHandle gh)
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

void ShyController::cancelCB(GoalHandle gh)
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

inline void ShyController::preemptActiveGoal()
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

  // Cancels the currently active goal
  if (current_active_goal)
  {
    rt_active_goal_.reset();
    current_active_goal->gh_.setCanceled();
  }
}

void ShyController::successActiveGoal()
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

// Visualization related things below 

void ShyController::publishTrajectoryMarkers(Eigen::MatrixXd& trajectory)
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
    Eigen::MatrixXd q_def = Eigen::MatrixXd::Zero(1, 7);
    for (int i = 0; i < trajectory.rows(); i++)
    {
      q_def = trajectory.row(i) - trajectory_deformation.row(i);
      forwardKinematics(q_def.transpose(), translation);
      marker.id = i;
      marker.pose.position.x = translation(0);
      marker.pose.position.y = translation(1);
      marker.pose.position.z = translation(2);
      if (i == deformed_segment_length+slow_index) {   // deformation horizon
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
    if (slow_index == trajectory_length - 1) markers.markers.clear(); // clear markers if trajectory is finished
    marker_publisher_->msg_ = markers;
    marker_publisher_->unlockAndPublish();

  }
}

void ShyController::fillFullTrajectoryMarkers(Eigen::MatrixXd& trajectory, int frequency)
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
  for (int i = 0; i < trajectory.rows(); i+=frequency)
  {
    forwardKinematics(trajectory.row(i).transpose(), translation);
    marker.id = i;
    marker.pose.position.x = translation(0);
    marker.pose.position.y = translation(1);
    marker.pose.position.z = translation(2);
    markers.markers.push_back(marker);
  }
  full_trajectory_markers_.markers.clear();
  full_trajectory_markers_ = markers;
}

Eigen::Matrix<double, 8, 4> ShyController::dh_params(const Eigen::Matrix<double, 7, 1>& joint_variable) 
{
  // Create DH parameters (data given by maker franka-emika)
  dh <<   0,      0,        0.333,   joint_variable(0, 0),
       -M_PI/2,   0,        0,       joint_variable(1, 0),
        M_PI/2,   0,        0.316,   joint_variable(2, 0),
        M_PI/2,   0.0825,   0,       joint_variable(3, 0),
       -M_PI/2,  -0.0825,   0.384,   joint_variable(4, 0),
        M_PI/2,   0,        0,       joint_variable(5, 0),
        M_PI/2,   0.088,    0.107,   joint_variable(6, 0),
        0,        0,        0.103,   -M_PI/4;

  return dh;
}

Eigen::Matrix4d ShyController::TF_matrix(int i, const Eigen::Matrix<double, 8, 4>& dh) 
{
  double alpha = dh(i, 0);
  double a = dh(i, 1);
  double d = dh(i, 2);
  double q = dh(i, 3);

  Eigen::Matrix4d TF;
  TF << cos(q), -sin(q), 0, a,
        sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d,
        sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d,
        0, 0, 0, 1;
  return TF;
}

void ShyController::forwardKinematics(const Eigen::Matrix<double, 7, 1>& joint_pose, Eigen::Vector3d& translation)
{
  Eigen::Matrix<double, 8, 4> dh_parameters = dh_params(joint_pose);

  Eigen::Matrix4d T_01 = TF_matrix(0, dh_parameters);
  Eigen::Matrix4d T_12 = TF_matrix(1, dh_parameters);
  Eigen::Matrix4d T_23 = TF_matrix(2, dh_parameters);
  Eigen::Matrix4d T_34 = TF_matrix(3, dh_parameters);
  Eigen::Matrix4d T_45 = TF_matrix(4, dh_parameters);
  Eigen::Matrix4d T_56 = TF_matrix(5, dh_parameters);
  Eigen::Matrix4d T_67 = TF_matrix(6, dh_parameters);
  Eigen::Matrix4d T_7E = TF_matrix(7, dh_parameters);

  Eigen::Matrix4d T_0E = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * T_67 * T_7E;

  translation = Eigen::Block<Eigen::Matrix4d, 3, 1>(T_0E, 0, 3);
}


}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ShyController,   //
                       controller_interface::ControllerBase)
