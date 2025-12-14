#include <ros/ros.h>
#include <std_msgs/String.h>

// Custom Message Declaration 
#include "anafi_ros1/traj.h"
#include "anafi_ros1/spData.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_speed");
    ros::NodeHandle nh;

    ros::Publisher pub=nh.advertise<anafi_ros1::spData>("/anafi/spData",100);

    ros::Rate rate(0.5);

    anafi_ros1::spData speed;

    speed.xpos = 0.0;
    speed.ypos = 0.0;
    speed.zpos = 0.0;

    speed.xvel = 50.0;
    speed.yvel = 50.0;
    speed.zvel = 50.0;

    while(ros::ok()){
        
        std::cout << speed << "\n";
        pub.publish(speed);
        rate.sleep();
    }


}