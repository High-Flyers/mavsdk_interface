#include <ros/ros.h>
#include <cstdint>
#include <iostream>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk_interface/gpsPos.h>
#include <mavsdk_interface/battery.h>
#include <mavsdk_interface/velocityNedPos.h>
#include <mavsdk_interface/flightMode.h>
#include <nav_msgs/Odometry.h>
#include "InformationDistributor.h"
#include "mavsdkUtils.hpp"

#define ROS_RATE 30

int main(int argc, char** argv)
{
    if(argc != 2)
    {
        std::cerr << "SPECIFY THE BIND ADDRESS\n";
        return 1;
    }

    ros::init(argc, argv, "t_mavsdk_node");
    ros::NodeHandle nh;

    ros::Publisher posGPS_pub = nh.advertise<mavsdk_interface::gpsPos>("mavsdk/gpsPos", 100);
    ros::Publisher battery_pub = nh.advertise<mavsdk_interface::battery>("mavsdk/battery", 100);
    ros::Publisher odo_pub = nh.advertise<nav_msgs::Odometry>("mavsdk/odometry", 100);
    ros::Publisher velNedPos_pub = nh.advertise<mavsdk_interface::velocityNedPos>("mavsdk/velocityNedPos", 100);
    ros::Publisher flightMode_pub = nh.advertise<mavsdk_interface::flightMode>("mavsdk/flightMode", 100);
  
    ros::Rate loop_rate(ROS_RATE);
    
    mavsdk::Mavsdk mavsdk;

    auto system = ConnectToDrone(mavsdk, argv[1]);
    if(system == nullptr)
        return 1;

    auto telemetry = mavsdk::Telemetry{system};
    
    InformationDistributor distibutor;
    distibutor.subcribePosition(posGPS_pub, telemetry);
    distibutor.subcribeBattery(battery_pub, telemetry);
    distibutor.subcribeOdometry(odo_pub, telemetry);
    distibutor.subcribePositionVelocityNed(velNedPos_pub, telemetry);
    distibutor.subcribeFlightMode(flightMode_pub, telemetry);
    
    ros::spin();
    return 0;
}

