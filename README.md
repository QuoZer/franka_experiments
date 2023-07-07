# franka_experiments
Repo for Aalto University summer internship. 

## Files
- shy_controller.cpp A controller based on the "Trajectory Deformations from Physical Human-Robot Interaction" paper. Takes joint trajectory messages as input and tries to follow them adapting to human interaction. 
- cartesian_traject_controller.cpp A slightly modified default cartesian_pose_controller. Follows the interactive marker until the first position message is recieved from a separate topic. Then listens to the second topic only.
- trajectory_pub.cpp A node that publishes trajectory points to cartesian_traject_controller and tries to detect human interactions based on F_ext. Skips time 'interaction time' forward.

## Launchfiles
