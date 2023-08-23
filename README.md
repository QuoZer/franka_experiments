# franka_experiments
Repo for Aalto University summer internship on human-robot interaction. 

## Dependencies
Developed and tested on Ubuntu 20.04 with libfranka 0.10.0 and ROS Noetic. For moveit integration the panda_moveit_config package is required.

## Nodes
- **shy_controller.cpp** - a controller based on the *"Trajectory Deformations from Physical Human-Robot Interaction"* paper. Takes joint trajectory messages as input and tries to follow them adapting to human interaction. Deformation visualization is available in *rviz*.
- **cartesian_traject_controller.cpp** - a slightly modified default *cartesian_pose_controller*. Follows the interactive marker until the first position message is recieved from a separate topic. Then listens to the second topic only.
- **trajectory_pub.cpp** - a node that publishes trajectory points to *cartesian_traject_controller* and tries to detect human interactions based on F_ext. Record the time someone interacts with a robot and thne skips to a waypoint 'interaction time' forward.
- **parameter_control.py** - a high-level script that uses python moveit interface to command *shy_controller* a trajectory and update parameters along the execution. 
- **move_group_direct_trajectory.py** - plans and executes a cartesian trajectory using *moveit* This trajectory is recorded in *1m_backforth.bag*.
- **traj_deform_demo.py** - demonstartes trajectory deformation on a 1 DoF trajectory example. 
- **/polymetis/** - experiments with Polymetis framework. 

## Launchfiles
- **gazebo_impendance_control.launch** - starts *certesian_trajectory_controller* in a simulated environment. Run trajectory_pub node to switch from interactive marker control to waypoint following. 
- **hw_cartesian_impendance_control.launch** - the same but with a real robot. 
- **gazebo_moveit_shy_control.launch** - starts *shy_controller* in a simulated environment using *moveit* interface. Plan trajectories and use dynamic reconfigure to adjust *admittance* and *deformation length* parameters. 
- **hw_moveit_shy_controller.launch** - integrates *shy_controller* with *moveit* interface and FCI.  

## Notes
Examples of the edited config files are available in *config/examples* folder.
- Edit **simple_moveit_controllers.yaml** to include *shy_controller* and *shy_cartesian_controller* in *panda_moveit_config* config files to run the controllers with moveit. Moveit is going to try to connect with all the controllers listed in the file though, so for faster bringup it's better to leave only the needed controller(s).  
- Edit **franka_control_node.yaml** at *franka_ros/franka_control/config/* to lower the external force thresholds.
- Edit **default_controllers.yaml** in *franka_control* package to set a higher update rate for *franka_state_controller*. It might be useful for parameter control. 
- Edit **franka_control.launch** in *panda_moveit_config* to comment out default controllers bringup.
- Disable *execution_duration_monitoring* in *trajectory_execution.launch.xml* to avoid timeouts.

Notable parameters for **shy_controller** testing: 
  - *admittance* - proportional term between the external force and the deformation amplitude.
  - *deformation_length* - the higher the longer is the deformation segment. 
  - *maximum_waypoint_distance / minimum_waypoint_count* of OMPL - set to increase trajectory resolution (doesn't seem to affect cartesian trajectories). 
  - *velocity_scaling* of motion planning in rviz - set to more comfortable for experiments values. 
  - *impendance gains* of the robot - the softer the gains the easier it is to push the robot around and the higher is the external force noise.  
  - External *force* and joint *torque* calm state values. The external force measurements are noisy and offset, so the controller might react to them even when the robot is not being pushed. These parameters remove the offset. 

## TODO
- [ ] External force autocorrection
- [ ] Merge *shy_cartesian_controller* and rename *shy_controller* to *shy_joint_controller*. 