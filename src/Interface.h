#ifndef Interface_h
#define Interface_h

#include <ros/ros.h>
#include <cstdint>
#include <iostream>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <exception>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <mavsdk_interface/gpsPos.h>
#include <mavsdk_interface/battery.h>
#include <mavsdk_interface/velocityNedPos.h>
#include <mavsdk_interface/flightMode.h>
#include <mavsdk_interface/arm.h>
#include <mavsdk_interface/takeoff.h>
#include <mavsdk_interface/kill.h>

#include "mavsdkUtils.hpp"
#include "InformationDistributor.h"

class Interface
{
private:
    mavsdk::Mavsdk mavsdk;
    InformationDistributor distibutor;

    ros::Publisher posGPS_pub;
    ros::Publisher battery_pub;
    ros::Publisher odo_pub;
    ros::Publisher velNedPos_pub;
    ros::Publisher flightMode_pub;

    std::shared_ptr<mavsdk::Telemetry> telemetry;
    std::shared_ptr<mavsdk::Action> action;

    ros::ServiceServer arm_srv;
    ros::ServiceServer takeoff_srv;
    ros::ServiceServer kill_srv;

public:
    Interface(ros::NodeHandle &nh, std::string udp);
    ~Interface();

    bool isArmed(mavsdk_interface::arm::Request &req, mavsdk_interface::arm::Response &res);
    bool takeoff(mavsdk_interface::takeoff::Request &req, mavsdk_interface::takeoff::Response &res);
    bool kill(mavsdk_interface::kill::Request &req, mavsdk_interface::kill::Response &res);
};

#endif