#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <fstream>          
#include <time.h>
#include <math.h>
#include <eigen3/Eigen/Dense>                
#include <eigen3/unsupported/Eigen/MatrixFunctions>

//ROS Headers
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

// OpenCV2 Header Files v   
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/highgui/highgui.hpp>

// Header Files Specific to the project
//#include"../include/color.h"
//#include"../include/twistops.hpp" 

// /tf Header Files 
#include<tf/transform_listener.h>
#include<tf/tf.h>
#include<tf/transform_datatypes.h>
                  
// Standard Message Declaration 
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>

// Nav Message Declaration 
#include <nav_msgs/Odometry.h>

// Custom Message Declaration 
#include "anafi_ros1/traj.h"
#include "anafi_ros1/spData.h"

//Declaration of Structures 
struct pid_gain
{
	Eigen::Vector4d Kp; 
	Eigen::Vector4d Ki;
	Eigen::Vector4d Kd; 
};

struct pid_var 
{
	Eigen::Vector4d er_vel;
	Eigen::Vector4d er_int;
	Eigen::Vector4d er_prev;

	tf::Vector3	pw;
	tf::Vector3 vw; 

	double yaw;
	double roll;
	double pitch;

	geometry_msgs::Twist control;
} ;

//Declaration of Global Variables 
Eigen::VectorXd sp_dat(7);

//Declaration of Subfunctions 
pid_gain i2d_gain(Eigen::Vector4i, Eigen::Vector4i, Eigen::Vector4i);
pid_var pid_controller(tf::StampedTransform, pid_gain, pid_var, Eigen::VectorXd, double , double);

void analysisFunc(ros::Publisher, anafi_ros1::traj, Eigen::VectorXd, pid_var);
void spData_Callback(anafi_ros1::spData); 
void moveFunc(ros::Publisher, geometry_msgs::Twist, double);

int main(int argc, char**argv)
{
    ros::init(argc,argv,"pidUAV_control");
    ros::NodeHandle nh; 

    // Open and Initialize Output File 
    std::ofstream outputFile;
    std::string filename = "Output.csv";

    outputFile.open(filename);
    outputFile << "Time (ros)"<<","<< "Time (s)" << "," << " x_w [cm]" <<","<< " y_w [cm]" << "," <<" z_w [cm]" << "," <<" yaw_w [rad]"<<"," << "pitch_w [rad]" << "," << "roll_w [rad]" << "," <<" x_vel_w [cm/s]" <<","<< " y_vel_w [cm/s]" <<","<<" z_vel_w [cm/s]" <<"," <<" x_ref [cm]" <<","<< " y_ref [cm]" << "," <<" z_ref [cm]" << "," <<" yaw_w [rad]"<<","<<" x_vel_w [cm/s]" <<","<< " y_vel_w [cm/s]"<<","<< " z_vel_w [cm/s]" <<std::endl;

	//Trackbar Declaration//

	//Trackbar Initialization 
	Eigen::Vector4i mKd = {3600,2200,1100,100};
	Eigen::Vector4i mKi = {62  ,60  ,30  ,70  }; 
	Eigen::Vector4i mKp = {4200 ,2100 ,3500 ,4000 };

	//Custom Variable Declaration 
	pid_gain pgain;	// PID Gains 
	pid_var pvar; // PID Variables 
	pvar.er_vel = Eigen::Vector4d::Zero();
	pvar.er_int = Eigen::Vector4d::Zero();
	pvar.er_prev = Eigen::Vector4d::Zero();

	// Create Trackbar 
    cv::namedWindow("PID Control and Destination");
    cv::createTrackbar("mKd_x","PID Control and Destination", &mKd[0], 10000);
    cv::createTrackbar("mKi_x","PID Control and Destination", &mKi[0], 100000);
    cv::createTrackbar("mKp_x","PID Control and Destination", &mKp[0], 10000);
        
    cv::createTrackbar("mKd_y","PID Control and Destination", &mKd[1], 10000);
    cv::createTrackbar("mKi_y","PID Control and Destination", &mKi[1], 100000);
    cv::createTrackbar("mKp_y","PID Control and Destination", &mKp[1], 10000);

    cv::createTrackbar("mKd_z","PID Control and Destination", &mKd[2], 10000);
    cv::createTrackbar("mKi_z","PID Control and Destination", &mKi[2], 100000);
    cv::createTrackbar("mKp_z","PID Control and Destination", &mKp[2], 10000);

    cv::createTrackbar("mKd_yaw","PID Control and Destination", &mKd[3], 10000);
    cv::createTrackbar("mKi_yaw","PID Control and Destination", &mKi[3], 100000);
    cv::createTrackbar("mKp_yaw","PID Control and Destination", &mKp[3], 10000);

	//Variable Declaration// 

	//Time Variables
    double del_t = 0.04; //Time Step 
    double tpres;        //Current Time
    double tprev = 0;    //Previous Time 
    double t;            //Relative Time 
    double dt = 0;       //Differential Time

	// Messages 
	geometry_msgs::Twist control; //Twist  
	anafi_ros1::traj tdat; //Visual Data Topic 

	// Intialize Variables 
	int i; 

    // Initialization of Desired Reference Trajectory 
    sp_dat = Eigen::VectorXd::Zero(7);	

	// ROS & /tf Declarations//	

	// /tf Variables 
    tf::TransformListener listener;
	tf::StampedTransform transform;   	

	// Publishers 
    ros::Publisher controller = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    ros::Publisher lmpc_pub   = nh.advertise<anafi_ros1::traj>("/lmpc_analysis",1);	

	// Subscribers 	
    ros::Subscriber sp_pub = nh.subscribe("/anafi/spData",1000,spData_Callback);  

	// Rate 
	ros::Rate rate(25);
    double t_ros = ros::Time::now().toSec();
	while (nh.ok())
	{
		//Update Trackbar 
		cv::waitKey(10);			
        
		//Read in Vicon Data 
        // try {
        //     listener.lookupTransform("/map","/vicon/bebop/bebop", ros::Time(0), transform);

            
        //     }
        // catch (tf::TransformException ex)
        // {
        //     ROS_ERROR("%s",ex.what());
        //     goto stop;        
        // }
		tf::Vector3 ptarget; 
		
		transform.setOrigin(tf::Vector3(0, 0, 0));

        if (sp_pub.getNumPublishers() < 1) 
        {
             ROS_ERROR("Node is not subscribing from the input node");
             
             goto stop;
        }

	    //Time Calculation
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
	
		// Convert from trackbar integers into usable doubles
		for (i = 0; i < 4; i++)
		{ 
			pgain.Kp[i] = ((double) mKp[i])/1000;
			pgain.Ki[i] = ((double) mKi[i])/10000;
			pgain.Kd[i] = ((double) mKd[i])/1000;
		}

		//PID Controller Code 
		pvar = pid_controller(transform, pgain, pvar, sp_dat, t, dt);	

		// Controller
		moveFunc(controller,pvar.control,t);

       // Read out to file
       outputFile <<t_ros<<","<<t<<","<<pvar.pw[0]<<","<<pvar.pw[1]<<","<<pvar.pw[2]<<","<<pvar.yaw<<","<<pvar.pitch<<","<<pvar.roll<<","<<pvar.vw[0]<<","<<pvar.vw[1]<<","<<pvar.vw[2] <<","<<sp_dat[0]<<","<<sp_dat[1]<<","<<sp_dat[2]<<","<<sp_dat[3]<<","<<sp_dat[4]<<","<<sp_dat[5]<<","<<sp_dat[6]<<std::endl;		

		analysisFunc(lmpc_pub, tdat, sp_dat, pvar);

		//Store Previous Information
		tprev = tpres;
		
		stop:
        // Spin the Node 
        ros::spinOnce(); 
        rate.sleep();
	}
	outputFile.close();
	return 0; 	
}

void spData_Callback(anafi_ros1::spData sp_topic) 
{
    sp_dat[0] = sp_topic.xpos;
    sp_dat[1] = sp_topic.ypos;
    sp_dat[2] = sp_topic.zpos;

    sp_dat[3] = sp_topic.yaw;

    sp_dat[4] = sp_topic.xvel;
    sp_dat[5] = sp_topic.yvel;
    sp_dat[6] = sp_topic.zvel;
    ROS_INFO("The input node is heard");
    
    
}

void moveFunc(ros::Publisher pub, geometry_msgs::Twist cmd, double t)
{
        // Publish Commands 
		pub.publish(cmd);
        ros::spinOnce();      

        // Power and Time Information Messages
        
        printf("\nx-direction: %f\n", cmd.linear.x);
        printf("y-direction: %f\n", cmd.linear.y);
        printf("z-direction: %f\n", cmd.linear.z);
        printf("Yaw: %f\n", cmd.angular.z);
        printf("Time [s]: %f\n", t);
}

pid_gain i2d_gain(Eigen::Vector4i mKp, Eigen::Vector4i mKi, Eigen::Vector4i mKd)
{
	// Intialize Variables 
	int i; 
	
	// PID Gain 
	pid_gain pgain;

	// Convert from trackbar integers into usable doubles
	for (i = 0; i < 4; i++)
	{ 
		pgain.Kp[i] = ((double) mKp[i])/1000;
		pgain.Ki[i] = ((double) mKi[i])/1000;
		pgain.Kd[i] = ((double) mKd[i])/1000;
	}	
	return pgain; 
}

pid_var pid_controller(tf::StampedTransform transform, pid_gain pgain, pid_var pvar_0 ,Eigen::VectorXd sp_data, double t, double dt)
{	
	
	//Custom Variable Declaration
	pid_var pvar_1;	

	//Rotation Declaration	
	double roll,pitch,yaw;
	double eyaw;	//Yaw Error

	//Rotation Matrix 
	tf::Matrix3x3 R; 

	R.setRotation(transform.getRotation());
	R.getRPY(roll,pitch,yaw);	
	pvar_1.yaw = yaw; // Store Yaw 
	pvar_1.pitch = pitch; //Store Pitch
	pvar_1.roll = roll; //Store Roll

	//Position Information 
	tf::Vector3 pw; 
	pw = transform.getOrigin(); 
	std::cout << "Position Information: " << pw.getX() << "\n";
	std::cout << "Position Information: " << pw.getY() << "\n";
	std::cout << "Position Information: " << pw.getZ() << "\n";
	std::cout << "------------------------";
	pvar_1.pw = pw; // Store (World Frame) Position 

	//Variable Declaration
	int i,j; 
	tf::Vector3 pdiff; //Position Error (World Frame) 
	tf::Vector3 epos;  //Position Error (Body Frame) 
	tf::Vector3 vel_w; //Velocity (World Frame)

	Eigen::Vector4d er;	//Total Error Storage 
	er = Eigen::Vector4d::Zero();

	//PID Error
	Eigen::Vector4d kep; 
	Eigen::Vector4d kei; 
	Eigen::Vector4d ked;
 
	//Error Calculation //
	for (i = 0; i < 3; i++)
	{
		pdiff[i] = sp_data[i] - pw[i];
		vel_w[i] = pdiff[i]/dt; 
	}
	pvar_1.vw = vel_w; // Store (World Frame) Velocity
	epos = R.transpose()*pdiff; // Position Error (Body Frame)

	eyaw = sp_data[3] - yaw;	// Rotation Error (World Frame) 

	//Rotation Adjustment 
	if (eyaw < (-1)*M_PI)
	{
		eyaw = eyaw + 2*M_PI;
	}
    else if (eyaw > M_PI)
	{
		eyaw = eyaw - 2*M_PI;
	} 

	//Error Storage 
	for (int k = 0; k < 4; k++)
	{
    	if (k < 3)
		{ 
        	er[k] = epos[k];
        }
        else if (k = 3)
		{
        	er[k] = eyaw;
        }
    }

    if( t == 0)
	{
		pvar_1.er_vel = Eigen::Vector4d::Zero();
	    pvar_1.er_int = Eigen::Vector4d::Zero();
	}
    else
    {
		for (int l = 0; l < 4; l++)
		{
			if (std::abs(er[l]) <= 0.01)
            { 
				kep[l] = 0;
            	kei[l] = 0;
            	ked[l] = 0;
            }
            else 
            { 
				//Proportional Error
				kep[l] = pgain.Kp[l]*er[l];

				//Integral Error 
				//pvar_0.er_int[l] += er[l]*dt;
				pvar_1.er_int[l] = pvar_0.er_int[l] + er[l]*dt;
                kei[l] = pvar_1.er_int[l]*pgain.Ki[l];

				//Differential Error 
                pvar_1.er_vel[l] = (er[l]-pvar_0.er_prev[l])/dt;
                ked[l] = pvar_1.er_vel[l]*pgain.Kd[l];    
            }
        }
          
	}     

	pvar_1.er_prev = er;
	pvar_1.control.linear.x = kep[0]+kei[0]+ked[0];	
    pvar_1.control.linear.y = kep[1]+kei[1]+ked[1];
    pvar_1.control.linear.z = kep[2]+kei[2]+ked[2];
    pvar_1.control.angular.z = kep[3]+kei[3]+ked[3];

	return pvar_1;
}

void analysisFunc(ros::Publisher lmpc_pub, anafi_ros1::traj traj, Eigen::VectorXd sdat,	pid_var pvar )
{
  
    traj.xtraj = pvar.pw[0];
    traj.ytraj = pvar.pw[1];
    traj.ztraj = pvar.pw[2];

    traj.yaw   = pvar.yaw;
	traj.pitch = pvar.pitch;
	traj.roll  = pvar.roll;

    traj.xtraj_vel = pvar.vw[0];
    traj.ytraj_vel = pvar.vw[1];
    traj.ztraj_vel = pvar.vw[2];

    traj.xtraj_pred = sdat[0];
    traj.ytraj_pred = sdat[1];

    traj.ztraj_pred = sdat[2];
    traj.yaw_pred   = sdat[3];
    traj.xtraj_pred_vel = sdat[4];
    traj.ytraj_pred_vel = sdat[5];
    traj.ztraj_pred_vel = sdat[6];

    //Publish Simulation
    lmpc_pub.publish(traj);

    ros::spinOnce();
}
