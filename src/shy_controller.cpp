// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
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
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_trajectory_ = node_handle.subscribe(
      "move_group/display_planned_path", 20, & ShyController::trajectoryCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

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

  if (!node_handle.getParam("trajectory_deformed_length", trajectory_deformed_length)) {
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
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(& ShyController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  return true;
}

void  ShyController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
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
  dq_d = dq_initial;    // zero?

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
  
  // Deformations precompute
  int N = trajectory_deformed_length;
  unit = Eigen::MatrixXd::Ones(N, 1);
  Uh = Eigen::MatrixXd::Zero(N, 1);
  dq_filtered_.fill(0);   // init with zeros
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

  B = Eigen::MatrixXd::Zero(4, N);
  B(0, 0) = 1;
  B(1, 1) = 1;
  B(2, N - 2) = 1;
  B(3, N - 1) = 1;

  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(N, N);
  G = (I - R.inverse() * B.transpose() * (B * R.inverse() * B.transpose()).inverse() * B ) * R.inverse() * unit ;    

  H = std::sqrt(N) * G / G.norm();    // check if norm is correct
  ROS_INFO("Finished precompute");

}

void  ShyController::update(const ros::Time& /*time*/,
                            const ros::Duration& /*period*/) {
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
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());


  // TRAJECTORY DEFORMATION
  if (haveTrajectory) {
    fast_index++;
    trajectory_sample_time = trajectory_times(slow_index+1, 0);
  }
  if (haveTrajectory && fast_index*loop_sample_time >= trajectory_sample_time)    // time system is not reliable 
  {
    slow_index++;
    fast_index = 0;
    //ROS_ASSERT(trajectory_sample_time > 0);
    
    //prev_time = pow(10, 9) * trajectory_times(slow_index, 0) + trajectory_times(slow_index, 1);
    //Eigen::Map<Eigen::Matrix<double, 6, 1>> fh(robot_state.K_F_ext_hat_K.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> uh(robot_state.tau_ext_hat_filtered.data());
    // parallize? 
    for (int dim = 0; dim < 7; dim++)
    {
      // calculate deformation for each joint separetely
      // Uh(slow_index) = uh(dim);   // Uh = (uh at the current time step | 0 at the rest)
      //deform_trajectory_positions.col(dim) = deform_trajectory_positions.col(dim) + trajectory_sample_time/pow(10, 9) * H * Uh.transpose();
      // Nx1 = Nx1 + 1x1 * Nx1 * Nx1.T
      deform_trajectory_positions.col(dim) = deform_trajectory_positions.col(dim) + admittance * trajectory_sample_time/pow(10, 9) * H * uh(dim);
      // Nx1 = Nx1 + 1x1 * Nx1 * 1x1
      // Uh(slow_index) = 0;
    }
    // update q_d and qd_d
    q_d = deform_trajectory_positions.row(0);
    delta_q = (deform_trajectory_positions.row(1) - deform_trajectory_positions.row(0)); 
    //ROS_INFO("delta_q values are: %f, %f, %f, %f, %f, %f, %f", delta_q(0), delta_q(1), delta_q(2), delta_q(3), delta_q(4), delta_q(5), delta_q(6));
    if (trajectory_sample_time == 0 || slow_index == trajectory_length - 1) 
      dq_d = Eigen::MatrixXd::Zero(7, 1).row(0);    // zero velocity at the start and end
    else 
      dq_d = delta_q * pow(10, 9) / trajectory_sample_time;   //nsec to sec
    //ROS_INFO("dq_d values are: %f, %f, %f, %f, %f, %f, %f", dq_d(0), dq_d(1), dq_d(2), dq_d(3), dq_d(4), dq_d(5), dq_d(6));
    // remove the first row and move data up
    deform_trajectory_positions.block(0, 0, deform_trajectory_positions.rows()-1, deform_trajectory_positions.cols()) = 
        deform_trajectory_positions.block(1, 0, deform_trajectory_positions.rows(), deform_trajectory_positions.cols());
    // add new new waypoint to the end
    if (slow_index+trajectory_deformed_length < trajectory_length)
      deform_trajectory_positions.row(trajectory_deformed_length-1) = trajectory_positions.row(slow_index+trajectory_deformed_length);
    else
      deform_trajectory_positions.row(trajectory_deformed_length-1) = trajectory_positions.row(trajectory_length-1);

    if (slow_index == trajectory_length - 1)
    {
      ROS_INFO("Trajectory execution finished after %d waypoints", slow_index);
      slow_index = 0;
      haveTrajectory = false;
    }
  }

  // honestly, don't know what this is for (from joint impedance example)
  double alpha = 0.99;
  dq_filtered_ = (1 - alpha) * dq_filtered_ + alpha * dq;
  // impedance control
  tau_d_calculated = coriolis_factor_ * coriolis +
                            k_gains_ * (q_d - q) +
                            d_gains_ * (dq_d - dq_filtered_);
      
  // saturation to avoid discontinuities
  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  tau_d_saturated = saturateTorqueRate(tau_d_calculated, tau_J_d);
  //ROS_INFO("tau_d_saturated 0 and 6: %f, %f", tau_d_calculated[0], tau_d_calculated[6]);
  
  // cartiesian example has nullspace stiffness as well, skipping for now

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }



  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  // cartesian_stiffness_ =
  //     filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  // cartesian_damping_ =
  //     filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  // nullspace_stiffness_ =
  //     filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  // std::lock_guard<std::mutex> position_d_target_mutex_lock(
  //     position_and_orientation_d_target_mutex_);
  // position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  // orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}

Eigen::Matrix<double, 7, 1>  ShyController::saturateTorqueRate(
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

void  ShyController::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
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
}

// void  ShyController::equilibriumPoseCallback(
//     const geometry_msgs::PoseStampedConstPtr& msg) {
//   std::lock_guard<std::mutex> position_d_target_mutex_lock(
//       position_and_orientation_d_target_mutex_);
//   //if (mode)     
//   position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
//   Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
//   orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
//       msg->pose.orientation.z, msg->pose.orientation.w;
//   if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
//     orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
//   }
// }

void  ShyController::trajectoryCallback(
    const moveit_msgs::DisplayTrajectory::ConstPtr& msg) {

  if (haveTrajectory){
    ROS_WARN("Received a new trajectory while the old one is still being executed. Ignoring the new trajectory");
    return;
  }

  if (!msg->model_id.empty() && msg->model_id != robot_model_)
    ROS_WARN("Received a trajectory to display for model '%s' but model '%s' was expected", msg->model_id.c_str(), robot_model_.c_str());

  // Save the trajectory
  trajectory_ = msg->trajectory[0].joint_trajectory;
  num_of_joints = trajectory_.joint_names.size();
  trajectory_length = trajectory_.points.size();
  trajectory_positions = Eigen::MatrixXd(trajectory_length, num_of_joints);
  trajectory_velocities = Eigen::MatrixXd(trajectory_length, num_of_joints);
  trajectory_times = Eigen::MatrixXi(trajectory_length, 1); 
  deform_trajectory_positions = Eigen::MatrixXd(trajectory_deformed_length, num_of_joints);
  //deform_trajectory_velocities = Eigen::MatrixXd(trajectory_deformed_length, num_of_joints);
  // probably can be done in a more efficient way
  int prev_ts = 0;
  for (int i = 0; i < trajectory_length; i++){
    for (int j = 0; j < num_of_joints; j++){
      trajectory_positions(i, j) = trajectory_.points[i].positions[j];
      trajectory_velocities(i, j) = trajectory_.points[i].velocities[j];
      // also copy the first N waypoints to trajectory_deform
      if (i < trajectory_deformed_length)
        deform_trajectory_positions(i, j) = trajectory_.points[i].positions[j];
    }
  // 						nsecs:  628257235       dt = 628257236
  // 						nsecs:  926514471       dt = 298257236
  // 	 1 sec		nsecs:  224771706       dt = 298257235
  //   1 sec 		nsecs:  523028942       dt = 298257236
  //   ...
  //	 6 sec 		nsecs:  891659179
  //   7 sec 	  nsecs:  519916415       dt = 628257236
    // 0 | 0 40 | 0 80 | 1 20 | 
    // 0 | 40 | 40 | 40
    trajectory_times(i, 0) = trajectory_.points[i].time_from_start.toNSec() - prev_ts;
    prev_ts = trajectory_.points[i].time_from_start.toNSec();
  }
  //trajectory_times(0, 0) = loop_sample_time; // HACK to avoid zero devision in the first iteration  
  haveTrajectory = true;
  ROS_INFO("Received a new trajectory with %d waypoints", trajectory_length);
} 


}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ShyController,   //
                       controller_interface::ControllerBase)
