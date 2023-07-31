# franka_experiments
Repo for Aalto University summer internship. 

## Files
- **shy_controller.cpp** - a controller based on the *"Trajectory Deformations from Physical Human-Robot Interaction"* paper. Takes joint trajectory messages as input and tries to follow them adapting to human interaction. Deformation visualization is available in *rviz*.
- **cartesian_traject_controller.cpp** - a slightly modified default *cartesian_pose_controller*. Follows the interactive marker until the first position message is recieved from a separate topic. Then listens to the second topic only.
- **trajectory_pub.cpp** - a node that publishes trajectory points to *cartesian_traject_controller* and tries to detect human interactions based on F_ext. Record the time someone interacts with a robot and thne skips to a waypoint 'interaction time' forward.
- **move_group_direct_trajectory.py** - plans and executes a cartesian trajectory  the trajectory recorded in *1m_backforth.bag*.
- **traj_deform_demo.py** - demonstartes trajectory deformation on a 1 DoF trajectory example. 
- **/polymetis/** - experiments with Polymetis framework. 

## Launchfiles
- **gazebo_impendance_control.launch** - starts *certesian_trajectory_controller* in a simulated environment. Run trajectory_pub node to switch from interactive marker control to waypoint following. 
- **hw_cartesian_impendance_control.launch** - the same but with a real robot. 
- **gazebo_moveit_shy_control.launch** - starts *shy_controller* in a simulated environment using *moveit* interface. Plan trajectories and use dynamic reconfigure to pick *admittance* and *deformation length* parameters. 
- **hw_moveit_shy_controller.launch** - integrates *shy_controller* with *moveit* interface and FCI.  

## Notes
- Edit **simple_moveit_controllers.yaml** to include *shy_controller* in *panda_moveit_config* package files to run on a real robot. TODO: Fix this
- Edit **franka_control.launch** in *panda_moveit_config* to comment out default controllers bringup.
- Disable *execution_duration_monitoring* in *trajectory_execution.launch.xml* to avoid timeouts
- For smoother perfomance change ompl dynamic parameters (maximum waypoint distance or minimum number of waypoints) to increase trajectory resolution. 
