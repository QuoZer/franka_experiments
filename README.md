# franka_experiments
Repo for Aalto University summer internship. 

## Files
- **shy_controller.cpp** - a controller based on the *"Trajectory Deformations from Physical Human-Robot Interaction"* paper. Takes joint trajectory messages as input and tries to follow them adapting to human interaction. (admittance 0.01 and trajectory_length of 36 show the effect a bit)
- **cartesian_traject_controller.cpp** - a slightly modified default *cartesian_pose_controller*. Follows the interactive marker until the first position message is recieved from a separate topic. Then listens to the second topic only.
- **trajectory_pub.cpp** - a node that publishes trajectory points to *cartesian_traject_controller* and tries to detect human interactions based on F_ext. Record the time someone interacts with a robot and thne skips to a waypoint 'interaction time' forward.
- **move_group_direct_trajectory.py** - plans and executes the trajectory recorded in *1m_backforth.bag*. 
- **/polymetis/** - experiments with Polymetis framework. 

## Launchfiles
- **gazebo_impendance_control.launch** - starts *certesian_trajectory_controller* in a simulated environment. Run trajectory_pub node to switch from interactive marker control to waypoint following. 
- **hw_cartesian_impendance_control.launch** - the same but with a real robot. 
- **gazebo_shy_control.launch** - starts *shy_controller* in a simulated environment. Use bagfiles to command trajectories and dynamic reconfigure to pick *admittance* and *deformation length* parameters. 
- **hw_shy_control.launch [WIP!]** - attempts to run *shy_controller* using FCI. Running it at the current stage may lead to *unforseen consequences*. 
- **hw_moveit_shy_controller.launch [WIP!]** - attmepts to integrate *shy_controller* with moveit interface. 

## Notes
- Edit **simple_moveit_controllers.yaml** in *panda_moveit_config* package files to run on a real robot. TODO: Fix this
