#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <franka_experiments/trajectory_pub.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "geometry_msgs_node");

    TrajectoryPubNode node;
    node.Start();

    return 0;
}



// void drive_to_start(double start_x, double start_y, double start_z, int samples)
// {
//     transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_EE", ros::Time(0));

//     double dx = start_x - transformStamped.transform.translation.x;
//     double dy = start_y - transformStamped.transform.translation.y;
//     double dz = start_z - transformStamped.transform.translation.z;
//     //double distance = sqrt(dx*dx + dy*dy + dz*dz);
//     //double velocity = distance / samples;
//     geometry_msgs::PoseStamped startPose;
//     startPose.header.frame_id = "panda_link0";
//     for (double progress = 1; ros::ok() && progress <= samples; progress += 1)
//     {
//         startPose.header.stamp = ros::Time::now();
//         startPose.pose.position.y = start_y + dy * progress / samples;
//         startPose.pose.position.x = start_x + dx * progress / samples;
//         startPose.pose.position.z = start_z + dz * progress / samples;
        
//         pub.publish(startPose);

//         ros::spinOnce();
//         r.sleep();
//     }    
// }

