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
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// actionlib
#include <actionlib/server/action_server.h>
#include <moveit_msgs/DisplayTrajectory.h>

// ROS messages
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

// realtime_tools
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_server_goal_handle.h>

// franka
#include <franka_experiments/cart_compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

namespace franka_example_controllers {

class  ShyCartesianController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  /* functions called by a controller manager */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time& /*time*/) override;

 private:
 /* Time storing data structure */
  struct TimeData
  {
    TimeData() : time(0.0), period(0.0), uptime(0.0) {}

    ros::Time     time;   ///< Time of last update cycle
    ros::Duration period; ///< Period of last update cycle
    ros::Time     uptime; ///< Controller uptime. Set to zero at every restart.
  };

/* Joint states data structure*/
  struct State
  {
    State() : position(7, 0.0), velocity(7, 0.0), acceleration(7, 0.0), time_from_start(0.0) {} 

    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    ros::Duration time_from_start;
  };
/* Type aliases */
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>                  ActionServer;
  typedef std::shared_ptr<ActionServer>                                                       ActionServerPtr;
  typedef ActionServer::GoalHandle                                                            GoalHandle;
  typedef trajectory_msgs::JointTrajectory::ConstPtr                                          JointTrajectoryConstPtr;
  typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction> RealtimeGoalHandle;
  typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;
  typedef realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray>                  MarkerPublisher;
  typedef std::unique_ptr<MarkerPublisher>                                                    MarkerPublisherPtr;

// Deforamtion related functions
  /*
  Parse and save a trajectory message into internal data structures 
  
    \param traj The received joint trajectory 
  */
  void parseTrajectory(const trajectory_msgs::JointTrajectory& traj);

  /* 
  Generates the trajectory deformation matrix based on the deformation length. 
  Takes rather long, avoid using in the control loop.

    \param N Length of the deformation vector
  */
  void precompute(int N);

  /* 
  Downsample the deformation vector to the desired length.

    \param new_N desired length of the new deformaton vector. Will be kept 
    between 10 and the trajectory length 
  */
  void downsampleDeformation(int new_N);

  /* 
  Perform the trajectory deformation and output next goal position and velocity 

    \param[in] robot_state The current tobot state 
    \param[out] q_d Desired deformed position is returned through this reference
    \param[out] dq_d Desired orientaion is returned through this reference 
  */
  void getDeformedGoal(franka::RobotState& robot_state,  
                       Eigen::Vector3d& position_d_, 
                       Eigen::Quaterniond& orientation_d_);

  /*
  Compute the desired torque based on the current robot state and the desired
  position. 

    \param robot_state The current robot state 
    \param tau_d The desired torque is returned through this reference 
  */
  void computeTau(franka::RobotState& robot_state, 
                  Eigen::VectorXd& tau_d);

  /* 
  Saturate to avoid discontinuities. Maximum torque difference with a sampling 
  rate of 1 kHz. The maximum torque rate is 1000 * (1 / sampling_time).
  
    \param  tau_d_calculated The calculated desired torque values 
    \param  tau_J_d Desired link-side joint torque sensor signals without gravity.
    \returns Saturated torque rate
  */
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

// Callbacks and action stuff
  /* Dynamic reconfigure CB */
  void complianceParamCallback(franka_experiments::cart_compliance_paramConfig& config,
                               uint32_t level);
  /* Trajectory message CB*/
  void trajectoryCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);
  /* Trajectory action CB */
  virtual void goalCB(GoalHandle gh);
  /* Cancel the active goal */
  virtual void cancelCB(GoalHandle gh);
  virtual void preemptActiveGoal();
  /* Set the active goal to SUCCESS state and drop it */
  void successActiveGoal();

  /* 
  Form and send action feedback  
  
  \param time_data Current time data structure
  \param robot_state Current robot state
  \param q_d Desired joint positions
  \param dq_d Desired joint velocities
  */
  void setActionFeedback(State& desired_state, State& current_state);

// Viz markers
  /* 
  Send updated trajectory visualization (e.g. with deformation)
  
  \param trajectory Trajectory you want to visualize in addition to the not deformed one
  */
  void publishTrajectoryMarkers(Eigen::MatrixXd& trajectory, int frequency);

  /* 
  Fill the original undeformed trajectory marker vector (blue)

  \param trajectory The original trajectory
  \param frequency  Specify to downsample the trajectory markers for visulization
  */
  void fillFullTrajectoryMarkers(Eigen::MatrixXd& trajectory, int frequency);

  visualization_msgs::MarkerArray full_trajectory_markers_;
  Eigen::Matrix<double, 8, 4> dh;
  /* Calc DH matrix for the given configuration */
  Eigen::Matrix<double, 8, 4> dh_params(const std::vector<double>& joint_variable);
  /* Calc transformation matrix for the input joint */
  Eigen::Matrix4d TF_matrix(int i, const Eigen::Matrix<double, 8, 4>& dh);
  /* Get translation vector from the given configuration */
  void forwardKinematics(const std::vector<double>& joint_pose, Eigen::Vector3d& translation, Eigen::Vector4d& orientation);

// time structures
  realtime_tools::RealtimeBuffer<TimeData> time_data_;
  TimeData prev_time_data_;
  ros::Timer goal_handle_timer_;
  int loop_sample_time = 1000000;      // nsec
  double action_monitor_rate = 20.0;   // Hz

// Hardware interfaces
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  
// Trajectory action stuff 
  ActionServerPtr       action_server_;
  RealtimeGoalHandlePtr rt_active_goal_;     ///< Currently active action goal, if any.
  
// Trajectory deformation stuff
  std::string robot_model_ = "panda";
  bool have_trajectory = false; 
  bool need_recompute = true;
  int num_of_joints = 7;
  int trajectory_sample_time = 0;       // delta, nsecs
  int trajectory_length = 0;            // samples  
  int fast_index = -1;                  // index of the fast update loop
  int slow_index = -1;                  // index of the slow (trajectory waypoint) update loop
  franka::RobotMode robot_mode;         // store the current robot mode

// trajectory message
  trajectory_msgs::JointTrajectory trajectory_;
  JointTrajectoryConstPtr trajectory_ptr_;

// trajectory eigen data structures
  Eigen::MatrixXd trajectory_positions;
  Eigen::MatrixXd trajectory_orientations;
  Eigen::MatrixXi trajectory_times;

// deformed trajectory eigen data
  Eigen::MatrixXd trajectory_deformation;
  Eigen::MatrixXd orientation_deformation;
  Eigen::MatrixXd pos_segment_deformation;
  Eigen::MatrixXd ori_segment_deformation;

// trajectory deformation matrixes
  Eigen::MatrixXd A;                    // minimum jerk trajectory model matrix
  Eigen::MatrixXd R;
  Eigen::MatrixXd B;                    // waypoint paramtrization matrix
  Eigen::MatrixXd G;                    // trajectory deformation matrix        
  Eigen::MatrixXd H;                    // Shape of optimal variation
  Eigen::MatrixXd H_full;                    // Shape of optimal variation
  Eigen::MatrixXd unit;                 // unit vector Nx1
  Eigen::MatrixXd Uh;                   // zero Nx1 for u

// Desired state
  Eigen::Matrix<double, 7, 1> dq_filtered_;
  Eigen::Matrix<double, 7, 1> q_d, delta_q, dq_d;      // desired joint position and velocity  
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Matrix<double, 7, 1> tau_d_calculated;
  Eigen::Matrix<double, 7, 1> tau_d_saturated;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;

// PARAMETERS`
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  double filter_params_{0.005};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 6, 1> force_thresholds_vector;    // currently useless
  Eigen::MatrixXd k_gains_;
  Eigen::MatrixXd d_gains_;
  std::mutex admittance_mutex_;
  double admittance = 0;                    // nu  
  double admittance_target_ = 0;            // nu for dynamic reconf
  int deformed_segment_length = 5;            // N, number of samples
  double deformed_segment_ratio_target_ = 0.1; // N for dynamic reconf
  double deformed_segment_ratio = 0.1;        // N for dynamic reconf
  int time_scaling_factor = 1;                // HACK to reduce velocity
  double coriolis_factor_{1.0};


// ROS things
  ros::NodeHandle controller_nh_;
  ros::Subscriber sub_trajectory_;
  ros::Subscriber trajectory_command_sub_;
  MarkerPublisherPtr marker_publisher_; 

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_experiments::cart_compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
};

}  // namespace franka_example_controllers
