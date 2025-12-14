#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/bind.hpp>
#include <cmath>
#include <iostream>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_msgs/TFMessage.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
using namespace std;
#define _USE_MATH_DEFINES

//Declaration of Structures
struct Quaternion {
    double w, x, y, z;                      //declaring quaternion variables
};

struct EulerAngles {
    double roll, pitch, yaw;                //declaring eulerian variables
};

//Declaration of functions
//class coordinate_converter{
    
        ros::NodeHandle n;

    //public:
            ros::Publisher euler_pub = n.advertise<std_msgs::Float64MultiArray>("/euler_pub1/angles",1); //this is for publishing
            

        void callback(std_msgs::Float64MultiArray msg , Quaternion q){
                
                EulerAngles angles;
                //q = msg;

                // roll (x-axis rotation)
                double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
                double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
                angles.roll = std::atan2(sinr_cosp, cosr_cosp);

                // pitch (y-axis rotation)
                double sinp = 2 * (q.w * q.y - q.z * q.x);
                if (std::abs(sinp) >= 1)
                    angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
                else
                    angles.pitch = std::asin(sinp);

                // yaw (z-axis rotation)
                double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
                double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
                angles.yaw = std::atan2(siny_cosp, cosy_cosp);

                //Publishing the message
                euler_pub.publish(angles);

            }   
    

int main(int argc , char **argv) 
{
    //Initiate ROS
    ros::init(argc, argv, "euler_pub1");
    ros::NodeHandle n;
    ros::Subscriber quaternion_sub = n.subscribe("vicon/bebop/-bebop", 1000, callback); //this is for subscribing
    //coordinate_converter euler_pose;
    ros::spin();

    return 0;
}

        



    
