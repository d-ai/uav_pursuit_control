/*
 * Node: Pursuit_vicon.cpp
 
 * This node uses information of the Vicon motion capture system to determine 
 * the relative position of the target AR.Drone to the pursuer AR.Drone. This 
 * node will subscribe from the /tf topic which contains the position and 
 * orientation of both the target and pursuer drone. 
 *
*/

// Header Files //
// Ros Headers 
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <ros/ros.h>

// /tf Header Files 
#include<tf/transform_listener.h>
#include<tf/tf.h>
#include<tf/transform_datatypes.h>
#include<tf/transform_broadcaster.h>

//C++ I/O
#include <iostream>
#include <stdio.h>
#include <string>
#include <sstream>
#include <fstream> 

//Standard headers
#include <time.h>
#include <math.h>

//OpenCV headers 
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Eigen headers 
#include <eigen3/Eigen/Dense>                
#include <eigen3/unsupported/Eigen/MatrixFunctions>

//Header files specific to this project
//#include "../include/detect.hpp"
#include "../include/colour.h"
#include "../include/twistops.hpp"

//Custom Message Declaration 
#include <ar_drone_ros/spData.h>
#include <ar_drone_ros/relpos.h>

//define TEST_FPS to enable framerate printout
#ifdef TEST_FPS
    int startTime;
    int counter;
#endif

//Declaration of Structures
struct pose_info
{
    Eigen::VectorXd pose_r;
    tf::Vector3     p_w_prev;
	tf::Vector3		p_b_prev;
	tf::Vector3 	pb_b;
	tf::Vector3 	pb_w;	
	tf::Vector3 	p_rel_b; 

	double pitch_w;
	double roll_w;
	double yaw_w;
	double pitch_b;
	double roll_b;
	double yaw_b;
};

//Declaration of Subfunctions
pose_info rel_pose(tf::StampedTransform, tf::StampedTransform, double, tf::Vector3, pose_info);
void pubFunc(ar_drone_ros::spData, ros::Publisher, pose_info, double);
void relFunc(ar_drone_ros::relpos, ros::Publisher, pose_info);

int main( int argc, char** argv )
{
    ros::init(argc, argv, "UAVDetect_vicon");
    ros::NodeHandle nh;

    ROS_INFO("Starting...\n");
    ROS_INFO("OpenCV version: %s\n",CV_VERSION);

	    // Open and Initialize Output File 
    std::ofstream outputFile;
    std::string filename = "Output_vicon.csv";

    outputFile.open(filename);
    outputFile << "Time (s)" << "," << " x_w_black[cm]" <<","<< " y_w_black [cm]" << "," <<" z_w_black [cm]" << "," <<" yaw_w_black [rad]"<<"," <<"roll_w_black [rad]"<<","<< "pitch_w_black"<<","<< " x_b_black[cm]" <<","<< " y_b_black [cm]" << "," <<" z_b_black [cm]" << ","<< " x_w_white[cm]" <<","<< " y_w_white [cm]" << "," <<" z_w_white [cm]" << "," <<" yaw_w_white [rad]"<<"," <<"roll_w_white [rad]"<<","<< "pitch_w_white"<<","<< " x_b_white[cm]" <<","<< " y_b_white [cm]" << "," <<" z_b_white [cm]"<<","<< " x_rel[cm]" <<","<< " y_rel [cm]" << "," <<" z_rel [cm]" <<std::endl;

    // Time Variables //
    double del_t = 0.04; // Time Step
    double tpres; // Current Time 
    double tprev = 0; //Previous Time
    double t; // Relative Time
    double dt = 0; //Differential Time

    //Variable Storage 
    pose_info pinfo;
	pinfo.pose_r = Eigen::VectorXd::Zero(7);

    // Declaration of Message File //
    ar_drone_ros::spData sp_dat;
	ar_drone_ros::relpos rel_dat; 

    // /tf Variables //
    tf::TransformListener listener;

    tf::StampedTransform black_transform;
    tf::StampedTransform white_transform;

    // Publishers // 
    ros::Publisher sp_pub = nh.advertise<ar_drone_ros::spData>("/ar_drone_ros/spData",1000);
	ros::Publisher rel_pub = nh.advertise<ar_drone_ros::relpos>("/ar_drone_ros/relpos",1000);
    
    // Previous Value Storage
    tf::Vector3 p_w_prev{0,0,0};

    // Node Rate //
    ros::Rate rate(25);

    while(ros::ok())
    {
        try
	{
	    listener.lookupTransform("/map","/vicon/ARDrone_Black/ARDrone_Black",ros::Time(0),black_transform);
	    listener.lookupTransform("/map","/vicon/ARDrone_White/ARDrone_White",ros::Time(0),white_transform);
	}
	catch (tf::TransformException ex) 
	{
	    ROS_ERROR("%s",ex.what());
	    goto stop;
	}

	// Time Calculation //
        tpres = ros::Time::now().toSec();
        if (tprev == 0)
        {
            t = 0;

        }
        else if (tprev != 0)
        {
            dt = tpres - tprev;
            t += dt; 
        }

    // Reference Information Calculation 
    pinfo =  rel_pose(black_transform, white_transform, dt, p_w_prev, pinfo);  

    // Publish Information 
    pubFunc(sp_dat, sp_pub, pinfo, t);
	relFunc(rel_dat, rel_pub, pinfo);

	std::cout << t << std::endl;
	std::cout << pinfo.pb_w[0] << std::endl;
	std::cout << pinfo.pb_w[1] << std::endl;
	std::cout << pinfo.pb_w[2] << std::endl;

    outputFile << t << "," << pinfo.p_b_prev[0] <<","<< pinfo.p_b_prev[1] << "," << pinfo.p_b_prev[2] << "," << pinfo.yaw_b <<"," << pinfo.roll_b <<","<< pinfo.pitch_b <<","<< pinfo.pb_b[0] <<","<< pinfo.pb_b[1] << "," << pinfo.pb_b[2] << ","<< pinfo.p_w_prev[0]<<","<< pinfo.p_w_prev[1] << "," << pinfo.p_w_prev[2] << "," << pinfo.yaw_w <<"," << pinfo.roll_w <<","<< pinfo.pitch_w <<","<< pinfo.pb_w[0] <<","<< pinfo.pb_w[1] << "," << pinfo.pb_w[2] <<","<< pinfo.pose_r[0] <<","<< pinfo.pose_r[1] << "," << pinfo.pose_r[2] <<std::endl;
 
	// Store Previous Information //
	tprev = tpres;
    p_w_prev = pinfo.p_w_prev;
	stop:
	// Spin the Node // 
	ros::spinOnce();
	rate.sleep();
    }	
	outputFile.close();
	return 0;
}

pose_info rel_pose(tf::StampedTransform black_transform, tf::StampedTransform white_transform, double dt, tf::Vector3 p_w_prev, pose_info pinfo_0)
{
        //Initialize Constants and Variables
        int i;  //For-Loop Value 
        double yaw_w, pitch_w, roll_w; //Yaw,Pitch,Roll for /ARDrone_White frame
		double yaw_b, pitch_b, roll_b; //Yaw,Pitch,Roll for /ARDrone_Black frame

        tf::Matrix3x3	R_black, R_white, R_rel, R_ref; // Rotation Matrices 
		tf::Vector3	p_black, p_white, p_rel, p_ref; // Position Vectors in World Coordinates
		tf::Vector3 pb_b, pb_w; // Position Vectors in Body Frame Coordinates 
        tf::Vector3 p_offset(-2,0,0);// Offset Position Vector
		tf::Vector3 beta(0.1,0.1,0.5);
        tf::Vector3 v_white, v_white_fil;

        //Initialize Output 
        pose_info pinfo;
        pinfo.pose_r = Eigen::VectorXd::Zero(7);

        //Rotation between /map to /ARDrone_Black frame 
        R_black.setRotation(black_transform.getRotation());

        //Rotation between /map to /ARDrone_White frame
        R_white.setRotation(white_transform.getRotation());
        
        //Extract Roll, Pitch, and Yaw for /ARDrone_White Frame 
        R_white.getRPY(roll_w,pitch_w,yaw_w);

        //Extract Roll, Pitch, and Yaw for /ARDrone_Black Frame 
        R_black.getRPY(roll_b,pitch_b,yaw_b);

        //Rotation between /ARDrone_Black to /ARDrone_White frame
        //R_rel = R_black.transpose()*R_white;	

        //Position from /map to /ARDrone_Black frame 
		p_black = black_transform.getOrigin();

        //Position from /map to /ARDrone_White frame
		p_white = white_transform.getOrigin();

		//Body Position of /ARDrone_White Frame 
		pb_w = R_white.transpose()*p_white; 

		//Body Position of /ARDrone_Black relative to the /ARDrone_White Frame. 
		pb_b = R_white.transpose()*p_black;	

        //Reference Position 
        p_ref = p_white + R_white*p_offset;

        //Reference Velocity 
        for (i = 0; i < 3; i++)
        {
            if (dt != 0)
            {
              //  v_white[i] = (p_white[i] - p_w_prev[i])/dt;
				v_white[i] = 0; 
				v_white_fil[i] = pinfo_0.pose_r[i+4] - beta[i]*(pinfo_0.pose_r[i+4]-v_white[i]);


            }
            else if ( dt == 0 || std::isnan(p_white[i]) == true)
            {
                v_white[i] = 0;
				v_white_fil[i] = 0;
            }
           
			//Store World Position for Both Drones
            pinfo.p_w_prev[i] = p_white[i];
			pinfo.p_b_prev[i] = p_black[i];
			
			//Store Body Position for Both Drones relative to /ARDrone_White
			pinfo.pb_b[i] = pb_b[i];
			pinfo.pb_w[i] = pb_w[i];
			pinfo.p_rel_b[i] = pb_w[i] - pb_b[i];
			
			//Store Reference Position 
            pinfo.pose_r[i]   = p_ref[i];
            //pinfo.pose_r[i+4] = v_white_fil[i];
			pinfo.pose_r[i+4] = v_white[i];
        }
            pinfo.pose_r[3] = yaw_w;

			//Store Rotations for Both Drones
			pinfo.yaw_b = yaw_b;
			pinfo.pitch_b = pitch_b;
			pinfo.roll_b = roll_b;

			pinfo.yaw_w = yaw_w;
			pinfo.pitch_w = pitch_w;
			pinfo.roll_w = roll_w;
        
        return pinfo;
}  

void pubFunc(ar_drone_ros::spData sp_dat, ros::Publisher sp_pub, pose_info pinfo, double t)
{
    //Store Information into Message
    sp_dat.xpos = pinfo.pose_r[0];
    sp_dat.ypos = pinfo.pose_r[1];
    sp_dat.zpos = pinfo.pose_r[2];

    sp_dat.xvel = pinfo.pose_r[4];
    sp_dat.yvel = pinfo.pose_r[5];
    sp_dat.zvel = pinfo.pose_r[6];

    sp_dat.yaw  = pinfo.pose_r[3];

    sp_dat.t    = t;
    //Publish the Message
//    std::cout << pinfo.pose_r[0] << std::endl;
    std::cout << "Xpos_d"    << std::endl;
    std::cout << sp_dat.xpos << std::endl;
    std::cout << sp_dat.ypos << std::endl;
    std::cout << sp_dat.zpos << std::endl;
    std::cout << sp_dat.xvel << std::endl;
    std::cout << sp_dat.yvel << std::endl;
    std::cout << sp_dat.zvel << std::endl;
    std::cout << sp_dat.yaw << std::endl;


    sp_pub.publish(sp_dat);

    //Spin Node
    ros::spinOnce();

}

void relFunc(ar_drone_ros::relpos rel_dat, ros::Publisher rel_pub, pose_info pinfo)
{
    //Store Information into Message
    rel_dat.xrel = pinfo.p_rel_b[0];
    rel_dat.yrel = pinfo.p_rel_b[1];
    rel_dat.zrel = pinfo.p_rel_b[2];

    rel_pub.publish(rel_dat);

    //Spin Node
    ros::spinOnce();

}
