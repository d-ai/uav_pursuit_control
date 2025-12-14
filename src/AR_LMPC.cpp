/*
 * Node: AR_LMPC.cpp

 * Description:
 * This is current update version of the linear model predictive control code 
 * for the Parrot AR.Drone 2.0. This code subscribes from the /tf topic which 
 * contains the pose of the drone and the /ar_drone_ros/spData topic which 
 * contains desired reference pose. The code publishes onto the /cmd_vel which 
 * sends desired inputs into the system. This code requires a model of the drone 
 * system. The model used in this code is an eight state system that includes 
 * the x,y,z, yaw error and velocities. This code also includes an observer to 
 * estimate the velocities. 
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
#include"../include/colour.h"
#include"../include/twistops.hpp" 

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
#include "ar_drone_ros/spData.h"


//Declaration of Structures 
struct lmpc_mat
{
    Eigen::MatrixXd Adb;
    Eigen::MatrixXd Cdb;
    Eigen::MatrixXd Glmpc;

	Eigen::MatrixXd Ad;  
	Eigen::MatrixXd Bd; 
	Eigen::MatrixXd C_obs; 	
};

struct tstate 
{ 
    Eigen::VectorXd xw_prev; 
    Eigen::VectorXd x_t;
    Eigen::VectorXd rtraj;

	// Luenberger Values 
	Eigen::VectorXd x_hat; 
	Eigen::VectorXd x_hat_w; 

	double roll;
	double pitch;
	double yaw;
};

struct output
{
    Eigen::VectorXd x_t1;
    Eigen::Vector4d u_t;
};

//Declaration of Global Variables 
Eigen::VectorXd sp_dat(7);
Eigen::MatrixXd Ad; 
Eigen::MatrixXd Bd;
Eigen::MatrixXd C_obs;

Eigen::MatrixXd A_obs; 
Eigen::MatrixXd L;

//Declaration of Subfunctions
lmpc_mat lmpc_gain(int, double, Eigen::MatrixXd, Eigen::Matrix4d);
tstate pose(tf::StampedTransform, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, double, double, Eigen::Vector4d,Eigen::VectorXd,Eigen::VectorXd);
output sys_in(lmpc_mat, tstate, Eigen::Vector4d, int); 

Eigen::Matrix2d S02_mat(double);


void spData_Callback(ar_drone_ros::spData); 
void moveFunc(ros::Publisher, Twist, output);

int main( int argc, char **argv)
{
    // Intialize Node 
    ros::init(argc,argv,"lmpc_AR_ros");
    ros::NodeHandle nh;

    // Open and Initialize Output File 
    std::ofstream outputFile;
    std::string filename = "Output.csv";

    outputFile.open(filename);
    outputFile << "Time (s)" << "," << " x_w [cm]" <<","<< " y_w [cm]" << "," <<" z_w [cm]" << "," <<" yaw_w [rad]"<<"," <<"roll_w [rad]"<<","<< "pitch_w"<<","<<" x_vel_w [cm/s]" <<","<< " y_vel_w [cm/s]" << "," <<" z_vel_w[cm/s]" << ","<<" x_ref [cm]" <<","<< " y_ref [cm]" << "," <<" z_ref [cm]" << "," <<" yaw_ref [rad]"<<"," <<" x_vel_ref [cm/s]" <<","<< " y_vel_ref [cm/s]" <<","<< " z_vel_ref [cm/s]" <<","<< " u_x" <<","<< " u_y"<<","<< " u_z" <<","<< " u_yaw" <<","<< " x_b" <<","<< " y_b"<<","<< " z_b" <<","<< " yaw_b" << "," << "yaw"   <<std::endl;

    // MPC Variables //

    // Predication Horizon 
    int Np = 30; 

    // Variable Storage 
    lmpc_mat lmpc_var;

    tstate tpose;
	tpose.x_t  = Eigen::VectorXd::Zero(8);
  	tpose.xw_prev = Eigen::VectorXd::Zero(8);
	tpose.x_hat = Eigen::VectorXd::Zero(8);
	tpose.x_hat_w = Eigen::VectorXd::Zero(8);
  	tpose.rtraj = Eigen::VectorXd::Zero(8*Np);

    output control_out;

    // Tuning Matrices

    // State Model Tuning Parameters 
    Eigen::VectorXd Q_c;
    Q_c = Eigen::VectorXd::Zero(8);
    Q_c << 3.25,3.75,3.25,10,4.25,4.75,1.75,0;

    Eigen::MatrixXd Q;
    Q  = Eigen::MatrixXd::Zero(8,8);
    Q  = Q_c.asDiagonal();


    // Input Tuning Parameters 
    Eigen::Matrix4d R;
    R = Eigen::Vector4d(1,1,1,2).asDiagonal();

    //Time Variables//
    double del_t = 0.04; //Time Step 
    double tpres;       //Current Time
    double tprev = 0;       //Previous Time 
    double t;           //Relative Time 
    double dt = 0;      //Differential Time

    // Additional Variable Declaration // 
    
    // Messages 
    Twist control; //Twist 

    // Initialization of Desired Reference Trajectory 
    sp_dat = Eigen::VectorXd::Zero(7);

    //Previous Data Initialization
    Eigen::Vector4d u_prev;
    Eigen::VectorXd xw_prev;
    Eigen::VectorXd xb_prev;
    Eigen::VectorXd x_hat_0;
    Eigen::VectorXd x_hat_w0;

    u_prev = Eigen::Vector4d::Zero();
    xw_prev = Eigen::VectorXd::Zero(8);
    xb_prev = Eigen::VectorXd::Zero(8);
	x_hat_0 = Eigen::VectorXd::Zero(8); 
	x_hat_w0 = Eigen::VectorXd::Zero(8); 

    // Gain Calculation //
    lmpc_var = lmpc_gain(Np,del_t,Q,R);

	//Observer Gain Calculation
	L = Eigen::MatrixXd::Zero(8,4);

	L << -1.0093, 0.092359,	-0.004846, -0.063331,
		  0.1398, -1.056, 0.10814, -0.1352,
		  0.12572, 0.069799, -0.92301, -0.014243,
		 -0.054235,	0.016682, 0.097198,	-0.8507,
		 -7.6447, 1.2987, -2.0157, 0.77804,
		  0.37247, -7.8088, 1.6255,	-2.934,
		  2.1764, -0.21832,	-6.2684, -1.2025,
		 -0.59451, 1.4429, 1.1007, -4.3633;


	A_obs = Ad + L*C_obs;
	std::cout << A_obs << std::endl;	

    // TF Variables // 
    tf::TransformListener listener;
    tf::StampedTransform transform;

    // Publishers // 
    ros::Publisher controller = nh.advertise<Twist>("/cmd_vel",1);

    // Subscribers //
    ros::Subscriber sp_pub = nh.subscribe("/ar_drone_ros/spData",1000,spData_Callback);  
    //Rate 
    ros::Rate rate(25);

    while (nh.ok())
    {
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

		//Read in Vicon Data 
        try {
            listener.lookupTransform("/map","/vicon/ARDrone_Black/ARDrone_Black", ros::Time(0), transform);
            }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            goto stop;        
        }

        if (sp_pub.getNumPublishers() < 1) 
        {
             ROS_ERROR("Node is not subscribing from the lmpc_sp_ros node");
             goto stop;
        }

        // Current Pose 
        tpose = pose(transform,sp_dat,xw_prev,xb_prev,dt,Np,u_prev,x_hat_0,x_hat_w0);
        //tpose = pose(transform,sp_dat,tpose,lmpc_var,dt,Np,u_prev);

        // Current Input; 
       control_out = sys_in(lmpc_var,tpose,u_prev,Np);

        // Controller
        moveFunc(controller,control,control_out); 


       // Read out to file
       outputFile <<t<<","<<tpose.xw_prev[0]<<","<<tpose.xw_prev[1]<<","<<tpose.xw_prev[2]<<","<<tpose.xw_prev[3]<<","<<tpose.roll<<","<<tpose.pitch<<","<<tpose.xw_prev[4]<<","<<tpose.xw_prev[5]<<","<<tpose.xw_prev[6]<<","<<sp_dat[0]<<","<<sp_dat[1]<<","<<sp_dat[2]<<","<<sp_dat[3]<<","<<sp_dat[4]<<","<<sp_dat[5]<< "," << sp_dat[6] << "," << control_out.u_t[0] << "," << control_out.u_t[1] << "," << control_out.u_t[2] << "," << control_out.u_t[3] << "," <<  tpose.x_t[0] << "," <<  tpose.x_t[1] << "," <<  tpose.x_t[2] << "," <<  tpose.x_t[3] << "," << tpose.yaw << std::endl;

        // Store Previous Information
        tprev = tpres;
        u_prev = control_out.u_t;
	    xw_prev = tpose.xw_prev; 
        xb_prev = tpose.x_t;
		x_hat_0 = tpose.x_hat; 
		x_hat_w0 = tpose.x_hat_w;


        stop:
        // Spin the Node 
        ros::spinOnce(); 
        rate.sleep(); 
    } 
    outputFile.close();
    return 0;
}

//Subfunctions 
 lmpc_mat lmpc_gain(int N, double del_t, Eigen::MatrixXd Q, Eigen::Matrix4d R)
{
    // Declaration of Iterators 
    int l,n,m;
    int p = N;

    // Declaration of Output
    lmpc_mat lmpc_var;

    //Delaration of State Constants
    Eigen::Matrix4d beta; 
    Eigen::Matrix4d alpha;
    Eigen::Matrix4d gamma;

    beta  = Eigen::Vector4d(-0.4451,-0.8409,-2.594,-6.0853).asDiagonal();
    alpha = Eigen::Vector4d(2.6215,3.8786,2.9673,-18.2618).asDiagonal();

    // Declaration of MPC Matrics and Gain 
    Eigen::MatrixXd A,B,C;
    Eigen::MatrixXd M,exp_M;
    Eigen::MatrixXd Cd,Id;
    Eigen::MatrixXd A_tilda,B_tilda,C_tilda;

    Eigen::MatrixXd A_dbar;
    Eigen::MatrixXd C_dbar;
    Eigen::MatrixXd FT_dbar;
    Eigen::MatrixXd H_dbar; 
    Eigen::MatrixXd Q_dbar; 
    Eigen::MatrixXd R_dbar;
    Eigen::MatrixXd T_dbar;
    Eigen::MatrixXd G_lmpc;

    // Intialization of MPC Matrices and Gain
    A  = Eigen::MatrixXd::Zero(8,8);
    B  = Eigen::MatrixXd::Zero(8,4);
    C  = Eigen::MatrixXd::Zero(8,8);

    M  = Eigen::MatrixXd::Zero(12,12);

    Ad = Eigen::MatrixXd::Zero(8,8);
    Bd = Eigen::MatrixXd::Zero(8,4);
    Cd = Eigen::MatrixXd::Zero(8,8);
    Id = Eigen::MatrixXd::Identity(4,4);

    A_tilda = Eigen::MatrixXd::Zero(12,12);
    B_tilda = Eigen::MatrixXd::Zero(12,4);
    C_tilda = Eigen::MatrixXd::Zero(8,12);

    A_dbar = Eigen::MatrixXd::Zero(N*12,12); 
    C_dbar = Eigen::MatrixXd::Zero(N*12,N*4);

    Q_dbar = Eigen::MatrixXd::Zero(N*12,N*12);
    R_dbar = Eigen::MatrixXd::Zero(N*4,N*4);
    T_dbar = Eigen::MatrixXd::Zero(N*8,N*12);
   
    FT_dbar = Eigen::MatrixXd::Zero(8*N+12,4*N);
    H_dbar = Eigen::MatrixXd::Zero(N*4,N*4);
   
    G_lmpc = Eigen::MatrixXd::Zero(N*4,8*N+12);         

    // Substitutions and Calculations
    
    // Substitution and Calculations of State

    // Continous Time State Matrices 
    A.block<4,4>(0,4) = Eigen::MatrixXd::Identity(4,4)*del_t;
    A.block<4,4>(4,4) = beta*del_t;

    B.block<4,4>(4,0) = alpha*del_t;

    C.block<8,8>(0,0) = Eigen::MatrixXd::Identity(8,8);
    Ad = A.exp();

    M.block<8,8>(0,0) = A;
    M.block<8,4>(0,8) = B; 

    exp_M = M.exp();

    Bd = exp_M.block<8,4>(0,8);
    Cd = C;       
    // Block Substitustions into LMPC
    A_tilda.block<8,8>(0,0) = Ad;
    A_tilda.block<8,4>(0,8) = Bd;
    A_tilda.block<4,4>(8,4) = Id;

    B_tilda.block<8,4>(0,0) = Bd;
    B_tilda.block<4,4>(8,0) = Id; 

    C_tilda.block<8,8>(0,0) = Cd;

    for (m = 0; m < N ; m++)
    {
        if (m == 0) 
        {
        A_dbar.block<12,12>(12*m,0) = A_tilda;
        }
        else
        {
        A_dbar.block<12,12>(12*m,0) = A_tilda.pow(m+1);
        }


        Q_dbar.block<12,12>(12*m,12*m) = C_tilda.transpose()*Q*C_tilda;
        R_dbar.block<4,4>(4*m,4*m) = R;
        T_dbar.block<8,12>(8*m,12*m) = Q*C_tilda;

            for (n = 0; n < p; n++)
            {
                if ( n == 0) 
                {
                    C_dbar.block<12,4>(12*m+12*n,4*m) = B_tilda; 
                }
                else if ( n == 1) 
                {
                     C_dbar.block<12,4>(12*m+12*n,4*m) = A_tilda*B_tilda; 
                }

                else
                {
                     C_dbar.block<12,4>(12*m+12*n,4*m) = A_tilda.pow(n)*B_tilda;
                }
            }
        p = --p;
    } 
    H_dbar = C_dbar.transpose()*Q_dbar*C_dbar + R_dbar; 
    FT_dbar.block(0,0,12,4*N) = A_dbar.transpose()*Q_dbar*C_dbar;
    FT_dbar.block(12,0,8*N,4*N) = -1*T_dbar*C_dbar; 
    G_lmpc = -1*H_dbar.inverse()*FT_dbar.transpose(); 


    lmpc_var.Adb = A_dbar;
    lmpc_var.Cdb = C_dbar;
    lmpc_var.Glmpc = G_lmpc;

	//Store Discretized Matrices 
    C_obs  = Eigen::MatrixXd::Zero(4,8); 
    C_obs.block<4,4>(0,0) = Eigen::MatrixXd::Identity(4,4); 
	std::cout << C_obs << std::endl;
	lmpc_var.Ad = Ad;
	lmpc_var.Bd = Bd;
	lmpc_var.C_obs = C_obs;		

  return lmpc_var;
}

tstate pose(tf::StampedTransform transform, Eigen::VectorXd sp_dat ,Eigen::VectorXd xw_prev ,Eigen::VectorXd xb_prev ,double dt, double Np, Eigen::Vector4d u_prev, Eigen::VectorXd x_hat_0, Eigen::VectorXd x_hat_w0) 
{
  	//Declaration of Output
	tstate pose;  
    
	Eigen::VectorXd x_hat_b0;
	x_hat_b0 = Eigen::VectorXd::Zero(8);

    Eigen::VectorXd x_t1;
    Eigen::Vector4d u_t;
	pose.x_t  = Eigen::VectorXd::Zero(8);
	pose.xw_prev = Eigen::VectorXd::Zero(8);
	pose.rtraj = Eigen::VectorXd::Zero(8*Np);
    pose.x_hat = Eigen::VectorXd::Zero(8); 
    pose.x_hat_w = Eigen::VectorXd::Zero(8); 
	

	//Declaration of Constants 
	int e,f,g,h,i,j,k,l;

	//Declaration of Variables
	double roll, pitch ,yaw;

	//Yaw Error Matrices  
	Eigen::Matrix2d Rt;
	Eigen::Matrix2d Re;
	Eigen::Matrix2d Rd;

	//Storage Matrices
	Eigen::Vector4d stat_prev;

	//Rotation Matrices 
	tf::Matrix3x3 R; 
	R.setRotation(transform.getRotation());
	R.getRPY(roll,pitch,yaw);
	R.setRPY(0,0,yaw);
	pose.pitch = pitch; 
	pose.roll = roll;
	pose.yaw = yaw;

	//Declare Positions and Velocity Variables
	tf::Vector3 vw, vb, vwr, vbr, vw_0, vb_0;
	tf::Vector3 pw, pb, pwr, pbr, pb_hat, pw_hat, pb_0,pw_0,pw_p, pb_p, vw_p, vb_p;

	//Current Pose Calculations 
	pw = transform.getOrigin();
	pb = R.transpose()*pw;
 
	Rt = S02_mat(yaw);
	Rd = S02_mat(sp_dat[3]);
	Re = Rd*Rt.transpose();

  	// Determing the Body Fixed Frame Reference Position of Drone
	for (j = 0; j < Np; j++)
	{
		for (k = 0 ; k < 3; k++)
    	{
        	pwr[k] = sp_dat[k];  
        	vwr[k] = sp_dat[k+4];
    	}
    
    	pbr = R.transpose()*pwr; 
    	vbr = R.transpose()*vwr;

    	pose.rtraj[0+8*j] = pbr[0];
    	pose.rtraj[1+8*j] = pbr[1];
    	pose.rtraj[2+8*j] = pbr[2];
    	pose.rtraj[3+8*j] = 0;
    	pose.rtraj[4+8*j] = vbr[0];
    	pose.rtraj[5+8*j] = vbr[1];
		pose.rtraj[6+8*j] = vbr[2];
		pose.rtraj[7+8*j] = 0;
	}

	// Leunberger Observer & Low-Pass Filtering 
  	for(f = 0; f < 3; f++) 
  	{
		pw_0[f] = x_hat_w0[f];
		vw_0[f] = x_hat_w0[f+4];

		pw_p[f] = xw_prev[f];
		vw_p[f] = xw_prev[f+4];
  	}
	pb_0 = R.transpose()*pw_0;
	vb_0 = R.transpose()*vw_0;

	pb_p = R.transpose()*pw_p;
	vb_p = R.transpose()*vw_p;

  	for(e = 0; e < 4; e++)
  	{ 	
		if (e < 3)
		{
			x_hat_b0[e] = pb_0[e];
			x_hat_b0[e+4] = vb_0[e];	
			
			stat_prev[e] = pb_p[e];
		} 
		else if ( e == 3)
		{
			x_hat_b0[e] = x_hat_w0[e];
			x_hat_b0[e+4] = x_hat_w0[e+4];
			stat_prev[e] = xw_prev[e];
		}
  	}	

 	if (dt != 0) 
  	{ 
    	pose.x_hat = A_obs*x_hat_b0 + Bd*u_prev - L*stat_prev; 

  		for(g = 0; g < 3; g++)
  		{
			vb[g] = pose.x_hat(g+4);
  		}
  	}
  	else if ( dt == 0 || std::isnan(pose.x_t[i]) == true)
  	{
     	for (h = 0; h < 4; h++)
	 	{ 
			if (h < 3)
			{
				pose.x_hat[h] = pb[h];
			} 
			else if ( h == 3) 
			{
				pose.x_hat[h] = std::atan2(Re(1,0),Re(0,0));
			}
	 	}
		vb.isZero();
 	}

	// Determing Body Fixed Frame Position of Drone and World Fixed Velocity of Drone
	for (i = 0; i < 3; i++) 
	{
		pb_hat[i] = pose.x_hat[i];		

    	pose.x_t[i] = pb[i];
		pose.x_t[i+4] = vb[i];

    	if (std::isnan(pose.x_t[i]) == true)
    	{
        	pose.x_t[i] =0;
    	}
	}
	pose.x_t[3] = std::atan2(Re(1,0),Re(0,0));
	pose.x_t[7] = pose.x_hat[7];

	// Determing Body Fixed Frame  Velocity of Drone 
    vw = R*vb;
    pw_hat = R*pb_hat;

    //Storage of Variables 
    pose.x_t[4] = vb[0];
    pose.x_t[5] = vb[1];
    pose.x_t[6] = vb[2];

    pose.xw_prev[0] = pw[0];
    pose.xw_prev[1] = pw[1];
    pose.xw_prev[2] = pw[2];
    pose.xw_prev[3] = pose.x_t[3];
    pose.xw_prev[4] = vw[0];
    pose.xw_prev[5] = vw[1];
    pose.xw_prev[6] = vw[2];
	pose.xw_prev[7] = pose.x_t[7];

	//Store World x_hat
	pose.x_hat_w[0] = pw_hat[0];
	pose.x_hat_w[1] = pw_hat[1];
	pose.x_hat_w[2] = pw_hat[2];
	pose.x_hat_w[3] = pose.x_t[3];
	pose.x_hat_w[4] = vw[0];
    pose.x_hat_w[5] = vw[1];
    pose.x_hat_w[6] = vw[2];	
	pose.x_hat_w[7] = pose.x_t[7];

   return pose;
}

output sys_in(lmpc_mat lmpc_var, tstate pose, Eigen::Vector4d u_prev , int Np)  
{
    //Declaration of Constants 
    int i;
    //Declaration of Variables
    Eigen::VectorXd xtr_vec;
    Eigen::VectorXd x_vec;
    Eigen::VectorXd x_tf;
    Eigen::VectorXd del_u;

    output control_out;

    xtr_vec = Eigen::VectorXd::Zero(Np*8+12);
    x_vec = Eigen::VectorXd::Zero(12);
    x_tf = Eigen::VectorXd::Zero(12*Np);

    del_u = Eigen::VectorXd::Zero(Np*4);

    //Substitution
    xtr_vec.head(8) = pose.x_t;
    xtr_vec.segment(8,4) = u_prev; 
    xtr_vec.segment(12,Np*8) = pose.rtraj;

    x_vec.head(8) = pose.x_t;
    x_vec.segment(8,4) = u_prev;

    //Calculation
    del_u = lmpc_var.Glmpc*xtr_vec;
    x_tf = lmpc_var.Cdb*del_u + lmpc_var.Adb*x_vec; 

    control_out.x_t1 = x_tf.head(8);
    control_out.u_t = x_tf.segment(8,4);

    for (i = 0; i < 4; i++)
    {
        if (control_out.u_t[i] < -1)
        {
            control_out.u_t[i] = -1;
        }      
        else if (control_out.u_t[i] > 1)
        {
            control_out.u_t[i] = 1;
        }
    }   

return control_out;
}

void moveFunc(ros::Publisher pub, Twist cmd, output control_out)
{
    //Commands 
    cmd.linear.x = control_out.u_t[0];
    cmd.linear.y = control_out.u_t[1];
    cmd.linear.z = control_out.u_t[2];
    cmd.angular.z = control_out.u_t[3];

    //Publish Commands 
    pub.publish(cmd);
    ros::spinOnce();    
}

void spData_Callback(ar_drone_ros::spData sp_topic) 
{
    sp_dat[0] = sp_topic.xpos;
    sp_dat[1] = sp_topic.ypos;
    sp_dat[2] = sp_topic.zpos;

    sp_dat[3] = sp_topic.yaw;

    sp_dat[4] = sp_topic.xvel;
    sp_dat[5] = sp_topic.yvel;
    sp_dat[6] = sp_topic.zvel;
    ROS_INFO("The lmpc_sp_ros node is heard");
}

Eigen::Matrix2d S02_mat (double yaw) 
{ 
    Eigen::Matrix2d S02; 
    S02(0,0) = std::cos(yaw);
    S02(0,1) = (-1)*std::sin(yaw);
    S02(1,0) = std::sin(yaw);
    S02(1,1) = std::cos(yaw);
    
    return S02;
}

