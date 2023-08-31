# franka_experiments
Repo for Aalto University summer internship on human-robot interaction. Additional notes are available in the NOTES.md file.

## Dependencies
Developed and tested on Ubuntu 20.04 with libfranka 0.10.0 and ROS Noetic. 
- [libfranka & franka_ros](https://frankaemika.github.io/docs/installation_linux.html)
- [Moveit 1](https://moveit.ros.org/install/)
- [panda_moveit_config](https://github.com/ros-planning/panda_moveit_config.git) (for better perfomance some config files need to be edited, see [Notes](#Notes) section)

## Trajectory Deformation Controller
The controller accepts trajectories (e.g. from moviet interface) and tracks them with impedance control. Human interactions are used to smoothly deform the trajectory. The controller is based on the *"Trajectory Deformations from Physical Human-Robot Interaction"* paper. Implemented in joint-space (*shy_joint_controller*) and task-space (*shy_cartesian_controller*) versions. Deformations can be visualized in *rviz*.

To run the controller in a simulated environment:
```bash
roslaunch franka_experiments gazebo_moveit_shy_control.launch controller:=shy_joint_controller  # or shy_cartesian_controller
```
To run the controller with a real robot:
```bash
roslaunch franka_experiments hw_moveit_shy_controller.launch controller:=shy_joint_controller  # or shy_cartesian_controller
```
**parameter_control.py** - a high-level script that uses python moveit interface to send *shy_controller* goals and update parameters during the execution based on the robot state. 
  ```bash
  rosrun franka_experiments parameter_control_demo.py
  ```
**traj_deform_demo.py** - a simple python program that demonstartes trajectory deformation on a 1 DoF trajectory example. 



## Notes
Consider editing these files before your hardware experiments.  Examples of the edited config files are available in *config/examples* folder.
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

## Other experiments

- **cartesian_traject_controller.cpp** - a slightly modified default *cartesian_pose_controller*. Follows an interactive marker until the first position message is recieved from a separate topic. Then listens to the second topic only.

  - **gazebo_timeskip_control.launch** - starts *certesian_trajectory_controller* in a simulated environment. Run trajectory_pub node to switch from interactive marker control to cartesian trajectory following. The trajectory can be published using *trajectory_pub* node. 
  - **hw_timeskip_control.launch** - the same but with a real robot. 
  - **trajectory_pub.cpp** - a node that publishes trajectory points to *cartesian_traject_controller* and tries to detect human interactions based on F_ext. Record the time someone interacts with a robot and thne skips to a waypoint 'interaction time' forward.

- **move_group_direct_trajectory.py** - plans and executes a cartesian trajectory using *moveit*. This trajectory is recorded in *1m_backforth.bag*.
- **/polymetis/** - experiments with [Polymetis](https://facebookresearch.github.io/fairo/polymetis/installation.html) framework. 

## TODO
- [ ] External force correction
- [ ] Adapt parameter control to the cartesian controller (topics are hardcoded now)
- [ ] k_gains / d_gains dynamic reconfigure
