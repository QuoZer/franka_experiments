#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <franka_experiments/trajectory_pub.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


TrajectoryPubNode::TrajectoryPubNode(int rate):
                nh{},
                pub{nh.advertise<geometry_msgs::PoseStamped>("cartesian_traject_controller/trajectory_pose", 10)},
                r{ros::Rate(rate)},
                tfBuffer{ros::Duration(1, 0)},
                tfListener{tfBuffer}
{  }

int TrajectoryPubNode::Start()
{
    ros::Duration(1.0).sleep(); // waiting to build up tf cache
    ROS_INFO("Moving to start position");
    drive_to_start(fix_x, start_y, fix_z, 1000);

    ros::Duration(5.0).sleep();  // Wait for the robot to reach the start position

    // Fixed orintation
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = "panda_link0";  // Frame in which pose is defined
    poseStamped.pose.orientation.x =  0.99999;
    poseStamped.pose.orientation.y =  4.93169e-05;
    poseStamped.pose.orientation.z =  0.00035;
    poseStamped.pose.orientation.w =  2.58384e-06;  // Set orientation to 
    //poseStamped.pose.orientation.normalize();
    ROS_INFO("Moving in a straight line");
    for (double y = start_y; ros::ok() && y < distance+start_y; y += velocity / rate)
    {
        poseStamped.header.stamp = ros::Time::now();
        poseStamped.pose.position.y = y;
        poseStamped.pose.position.x = fix_x;
        poseStamped.pose.position.z = fix_z;
        
        this->pub.publish(poseStamped);

        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("Done");

    return 0;
}

//TODO: make samples proportional to distance 
void TrajectoryPubNode::drive_to_start(double start_x, double start_y, double start_z, int samples)
{
    // transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_EE", ros::Time(0));
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_EE", ros::Time(0), 
                                                    ros::Duration(3.0));
        //transformStamped = tfBuffer.waitForTransform("panda_link0", "panda_EE", ros::Time(0), 
        //                                            ros::Duration(3.0));                   
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT find transform: %s", ex.what());
    }
    double cur_x = transformStamped.transform.translation.x;
    double cur_y = transformStamped.transform.translation.y;
    double cur_z = transformStamped.transform.translation.z;
    double dx = start_x - cur_x;
    double dy = start_y - cur_y;
    double dz = start_z - cur_z;
    //double distance = sqrt(dx*dx + dy*dy + dz*dz);
    //double velocity = distance / samples;
    geometry_msgs::PoseStamped startPose;
    startPose.header.frame_id = "panda_link0";
    for (double progress = 1; ros::ok() && progress <= samples; progress += 1)
    {
        startPose.header.stamp = ros::Time::now();
        startPose.pose.position.y = cur_y + dy * progress / samples;
        startPose.pose.position.x = cur_x + dx * progress / samples;
        startPose.pose.position.z = cur_z + dz * progress / samples;
        
        pub.publish(startPose);

        ros::spinOnce();
        r.sleep();
    }
}

TrajectoryPubNode::~TrajectoryPubNode() {}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_pub_node");
    ROS_INFO("Starting node");
    TrajectoryPubNode node{100};
    node.Start();

    return 0;
}
