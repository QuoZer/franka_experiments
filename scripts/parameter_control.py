from __future__ import print_function
from six.moves import input
import sys
import copy
import math
import numpy as np

import rospy
import dynamic_reconfigure.client

import geometry_msgs.msg
from std_msgs.msg import String
import franka_msgs.msg as franka

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))



def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class ShyControllerParameterInterface(object):
    """ShyControllerParameterInterface"""

    def __init__(self, velocity_factor):
        super(ShyControllerParameterInterface, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("shy_controller_parameter_interface", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints). This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        self.parameter_updater = dynamic_reconfigure.client.Client("/shy_controller/dynamic_reconfigure_compliance_param_node",
                                                              timeout=30,
                                                              config_callback=None)

        self.robot_state_subscriber = rospy.Subscriber("franka_state_controller/franka_states",
                                                   franka.FrankaState, 
                                                   self.state_callback)

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Parameters
        move_group.set_max_velocity_scaling_factor(velocity_factor)

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names



    def state_callback(self, data: franka.FrankaState):
        self.last_state = data
        
        return
    
    def update_controller_parameters(self, admittance, deform_length):
        # Update the parameters
        assert 0 <= admittance <= 0.5, "Admittance must be positive and not too high"
        assert 0 <= deform_length <= 1, "Deformed length must be between 0 and 1"
        
        self.parameter_updater.update_configuration({"admittance": admittance, "deformed_length": deform_length})

        return
        

    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = -0.03505
        pose_goal.orientation.x =  0.90729
        pose_goal.orientation.y = -0.41821
        pose_goal.orientation.z =  0.02635
        pose_goal.position.x = 0.61
        pose_goal.position.y = -0.03
        pose_goal.position.z = 0.63

        move_group.set_max_velocity_scaling_factor(0.02)
        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)

        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        print("Stopping trajectory execution and parameter update")
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.y += scale * 0.5  # 
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 1.0  # 
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y += scale * 0.5  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        move_group.set_max_velocity_scaling_factor(0.01)  # doesn't seem to work
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction
    
    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

def main():
    try:
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")

        interface = ShyControllerParameterInterface(velocity_factor=0.01)
        move_group = interface.move_group

        ## Step 1. Set a joint/pose/waypoint goal
        pose_goal = geometry_msgs.msg.Pose()        # a bit ti the front of the home position
        pose_goal.orientation.w = -0.03505
        pose_goal.orientation.x =  0.90729
        pose_goal.orientation.y = -0.41821
        pose_goal.orientation.z =  0.02635
        pose_goal.position.x =  0.61
        pose_goal.position.y = -0.03
        pose_goal.position.z =  0.63

        move_group.set_pose_target(pose_goal)

        # Waiting for callbacks
        rospy.sleep(1)

        ## Step 2. Execute the plan (non blocking)
        success = move_group.go(wait=False)

        ## Step 3. Update the parameters while the robot is moving
        i = 0
        while not all_close(pose_goal, move_group.get_current_pose().pose, 0.05):       # must be a better way to do this
            ## Step 3.1. Get the current state
            robot_state = interface.last_state # get the last state
            
            ## Step 3.2. Do smthng
            force = np.linalg.norm( np.array(robot_state.O_F_ext_hat_K[:3]) )
            #print(robot_state.O_F_ext_hat_K[:3])
            print("Force: ", force)
            calc_admittance = max(0, (force-6)/1000) 
            calc_deflength  = min(max(0.2, 5/force), 1)
            #  20 -> 0.2
            #   5 -> 1
            #   5/f
            print("Admm: {}; Length: {}".format(calc_admittance, calc_deflength) )
            
            ## Step 3.3. Update the parameters
            interface.update_controller_parameters( 0.002, calc_deflength )
            rospy.sleep(0.1)
            i+=1

        print(i)
        print("Stopping trajectory execution and parameter update")
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()


        print("============ demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()