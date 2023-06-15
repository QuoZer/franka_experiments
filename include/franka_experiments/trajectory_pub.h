#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class TrajectoryPubNode
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Rate r;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    /* data */
    double rate = 100.0;  // Hz
    double start_y = -0.5;
    double distance = 1.0;  // meters
    double velocity = 0.05;  // meters/second
    double fix_x = 0.3;
    double fix_z = 0.48;
public:
    TrajectoryPubNode(int rate/* args */);
    int Start();
    void drive_to_start(double start_x, double start_y, double start_z, int samples);
    ~TrajectoryPubNode();
};
