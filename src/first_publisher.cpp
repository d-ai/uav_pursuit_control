/*
 * Node: AR_PID.cpp
 * Author: Christopher Surma
 * Maintainer: Martin Barczyk
 * Date: Nov 27/19
 * Description:
 * This node is a PID controller that controls the position and yaw orientation 
 * of the AR.Drone using Vicon data. This code subscribes from the /ar_drone_ros
 * topic and publishes command inputs on the /cmd_vel topic. The PID gains are 
 * hard coded in this node and can be found below.
*/

// C++ I/O & Standard Headers

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <fstream>          
#include <time.h>
#include <math.h>


//ROS Headers
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>




// /tf Header Files 
#include<tf/transform_listener.h>
#include<tf/tf.h>
#include<tf/transform_datatypes.h>
                  
// Standard Message Declaration 
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>


int main(int argc, char**argv)
{
    ros::init(argc,argv,"pidUAV_control");
    ros::NodeHandle nh; 

    // // Open and Initialize Output File 
    // std::ofstream outputFile;
    // std::string filename = "Output.csv";



	// /tf Variables 
    tf::TransformListener listener;
	tf::StampedTransform transform;   	




	// Rate 
	ros::Rate rate(25);
    double t_ros = ros::Time::now().toSec();
	while (nh.ok())
	{
		

        listener.lookupTransform("/map","/vicon/anafi/anafi", ros::Time(0), transform);
        std::cout << transform.getOrigin();

        ros::spinOnce(); 
        rate.sleep();
	}


	return 0; 	
}
