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
from control_msgs.msg import FollowJointTrajectoryActionResult
from actionlib_msgs.msg import GoalStatusArray

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

    print("Goal: ", goal)
    print("Actual: ", actual)

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

    def __init__(self, velocity_factor, update_rate):
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
        # get the goal updates from the controller. Doesn't seem very reliable - update rate is around 5 Hz
        self.goal_state_subscriber = rospy.Subscriber("shy_controller/follow_joint_trajectory/status",
                                                     GoalStatusArray, 
                                                     self.goal_callback)

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
        self.move_group = move_group # group python class reference
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.goal_status = 3   # 1 - accepted, 3 - success, idle
        self.prev_goal_status = 3
        self.loop_rate = rospy.Rate(update_rate)


    def goal_callback(self, data: GoalStatusArray):
        if len(data.status_list) == 0:
            return
        
        self.prev_goal_status = self.goal_status
        self.goal_status = data.status_list[0].status

        return

    def state_callback(self, data: franka.FrankaState):
        self.last_state = data
        
        return
    
    def update_controller_parameters(self, admittance, deform_length):
        # Update the parameters
        assert 0 <= admittance <= 0.5, "Admittance must be positive and not too high"
        assert 0 <= deform_length <= 1, "Deformed length must be between 0 and 1"
        
        self.parameter_updater.update_configuration({"admittance": admittance, "deformed_length": deform_length})

        return
    
    def go(self, pose_goal, policy):
        self.move_group.set_pose_target(pose_goal)

        # Waiting for callbacks
        rospy.sleep(1)

        ## Step 2. Execute the plan (non blocking)
        success = self.move_group.go(wait=False)
        ## Step 2.1. Wait for start
        while self.goal_status != 1:
            self.loop_rate.sleep()

        ## Step 3. Update the parameters while the robot is moving
        i = 0
        while self.goal_status == 1:
        #not all_close(pose_goal, move_group.get_current_pose().pose, 0.05):       # must be a better way to do this
            ## Step 3.1. Get the current state
            robot_state = self.last_state # get the last state
            
            ## Step 3.2. Do smthng
            new_admittance, new_deflength = policy(robot_state)
            #print("Admm: {}; Length: {}".format(new_admittance, new_deflength) )
            
            ## Step 3.3. Update the parameters
            self.update_controller_parameters( 0.002, new_deflength )
            self.loop_rate.sleep()
            i+=1

        print(i)
        print("Stopping trajectory execution and parameter update")
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        self.move_group.clear_pose_targets()


        
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

def demo_rule(robot_state):
    force = np.linalg.norm( np.array(robot_state.O_F_ext_hat_K[:3]) )
    calc_admittance = max(0, (force-6)/1000) 
    calc_deflength  = min(max(0.2, 5/force), 1)
    #  20 -> 0.2
    #   5 -> 1
    #   5/f    
    return calc_admittance, calc_deflength


def main():
    try:
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")

        interface = ShyControllerParameterInterface(velocity_factor=0.05, update_rate=100)
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

        interface.go(pose_goal, demo_rule)      

        print("============ demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()