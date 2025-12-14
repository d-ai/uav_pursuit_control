#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

ros::Publisher euler_pub;
ros::Subscriber quat_sub;

// Function for conversion of quaternion to euler. The angles are published here too.
void Callback(geometry_msgs::TransformStamped msg)
{
    //converting transformstamped to quaternion
    geometry_msgs::Quaternion q = msg.transform.rotation;
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(q, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    // this Vector is then published:
    euler_pub.publish(rpy);
    ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", rpy.x, rpy.y, rpy.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "converter");
    ros::NodeHandle n;
    euler_pub = n.advertise<geometry_msgs::Vector3>("rpy_angles", 1000);
    quat_sub = n.subscribe("vicon/bebop/bebop", 1000, Callback);

    // check for incoming quaternions until ctrl+c is pressed
    ROS_INFO("waiting for quaternion");
    ros::spin();
    return 0;
}