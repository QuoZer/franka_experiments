#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <franka_experiments/trajectory_pub.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*
    

*/



TrajectoryPubNode::TrajectoryPubNode(int in_rate):
                nh{},
                rate(in_rate),
                pub{nh.advertise<geometry_msgs::PoseStamped>("cartesian_traject_controller/trajectory_pose", 10)},
                state_sub{nh.subscribe<geometry_msgs::WrenchStamped>("franka_state_controller/F_ext", 10, &TrajectoryPubNode::state_callback, this)},
                r{ros::Rate(rate)},
                tfBuffer{ros::Duration(1, 0)},
                tfListener{tfBuffer},
                window_size_{20} // 20ts 
{  }

int TrajectoryPubNode::Start()
{
    ros::Duration(1.0).sleep(); // waiting to build up tf cache
    ROS_INFO("Moving to the start position");
    drive_to_start(fix_x, start_y, fix_z, 0.1);

    ros::Duration(3.0).sleep();  // Wait for the robot to reach the start position

    // Fixed orintation
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = "panda_link0";  // Frame in which pose is defined
    poseStamped.pose.orientation.x =  0.99999;
    poseStamped.pose.orientation.y =  4.93169e-05;
    poseStamped.pose.orientation.z =  0.00035;
    poseStamped.pose.orientation.w =  2.58384e-06;  // Set orientation to 
    //poseStamped.pose.orientation.normalize();
    ROS_INFO("Moving in a straight line");
    double y = 0;
    double dy = velocity / rate;
    // 1 ts = 1/rate seconds
    for (int ts = 0; ros::ok() && y < distance+start_y; ts++)
    {
        if (interaction_ended)
        {
            interaction_ended = false;
            int dts = static_cast <int> (std::floor(total_time_*rate)); // skip to a goal 'interaction time' ahead
            ts += dts;
            ROS_INFO("Interaction ended, skipping %d timestamps", dts);
        }
        y = start_y + dy*ts;
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


void TrajectoryPubNode::drive_to_start(double start_x, double start_y, double start_z, double speed)
{
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_EE", ros::Time(0), 
                                                    ros::Duration(3.0));                 
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT find transform: %s", ex.what());
    }
    double cur_x = transformStamped.transform.translation.x;
    double cur_y = transformStamped.transform.translation.y;
    double cur_z = transformStamped.transform.translation.z;
    double dx = start_x - cur_x;
    double dy = start_y - cur_y;
    double dz = start_z - cur_z;
    double distance = sqrt(dx*dx + dy*dy + dz*dz);
    double samples = rate*distance / speed;
    ROS_INFO("in drive_to_start: distance = %f, samples = %f", distance, samples);
    geometry_msgs::PoseStamped startPose;
    startPose.header.frame_id = "panda_link0";
    for (int progress = 1; ros::ok() && progress <= samples; progress += 1)
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

void TrajectoryPubNode::state_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    F_ext = sqrt(msg->wrench.force.x*msg->wrench.force.x + 
                 msg->wrench.force.y*msg->wrench.force.y + 
                 msg->wrench.force.z*msg->wrench.force.z);   

    if (filter(F_ext) > F_threshold)
    {
        if (!above_threshold_)
        {
            // The value just went above the threshold
            above_threshold_ = true;
            start_time_ = ros::Time::now();
        }
    }
    else if (above_threshold_)
    {
        // The value just went below the threshold
        above_threshold_ = false;
        interaction_ended = true;
        total_time_ = (ros::Time::now() - start_time_).toSec();
    }
    

    //ROS_INFO("Total time above threshold: %f", total_time_);
}

double TrajectoryPubNode::filter(double new_value)
{
    if (window_.size() >= window_size_)
    {
        sum_ -= window_.front();
        window_.pop_front();
    }

    window_.push_back(new_value);
    sum_ += new_value;

    return sum_ / window_.size();
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
