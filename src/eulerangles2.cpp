#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cmath>
#include <iostream>
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

ros::NodeHandle n;
ros::Publisher euler_pub = n.advertise<std_msgs::Float64MultiArray>("/euler_pub2/euler",1000); //this is for publishing
std_msgs::Float64MultiArray EulerAngles;
EulerAngles callback(const std_msgs::Float64MultiArray& msg , Quaternion q)
void publish(EulerAngles angles, ros::Publisher , std_msgs::Float64MultiArray euler)

//Function for conversion
ros::Subscriber quaternion_sub = n.subscribe("vicon/bebop/-bebop", 1000,&coordinate_converter::callback, this); //this is for subscribing

    EulerAngles callback(std_msgs::Float64MultiArray EulerAngles angles , Quaternion q){
        Quaternion q = this;

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

        return angles;
    }


//Function for publishing

void publish(EulerAngles angles, ros::Publisher ,std_msgs::Float64MultiArray euler ){
    //Storing info in the message
    euler.roll = angles.roll;
    euler.pitch = angles.pitch;
    euler.yaw = angles.yaw;

    //Publishing the message
    cout << euler.roll << endl;
    cout << euler.pitch << endl;
    cout << euler.yaw << endl;

    euler_pub.publish(euler);

    //SpinNode
    ros::spinOnce();

}

//Main function

int main(int argc , char** argv){
    //Initiate ROS
    ros::init(argc, argv, "euler_pub2");
    ros::NodeHandle n;
    ros::Publisher euler_pub = n.advertise<std_msgs::euler>("/euler",1000); //this is for publishing
    
    ros::spin();

    return 0;
}

