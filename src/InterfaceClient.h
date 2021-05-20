#ifndef InterfaceClient_h
#define InterfaceClient_h

#include <ros/ros.h>
#include <cstdint>
#include <iostream>
#include <mavsdk/mavsdk.h>
#include <mavsdk_interface/arm.h>
#include <mavsdk_interface/takeoff.h>
#include <mavsdk_interface/kill.h>
#include <mavsdk_interface/go.h>
#include <chrono>
#include <thread>
#include "mavsdkUtils.hpp"

class InterfaceClient
{
private:
    ros::ServiceClient arm_clt;
    ros::ServiceClient takeoff_clt;
    ros::ServiceClient kill_clt;
    ros::ServiceClient go_clt;

    mavsdk_interface::arm arm_srv;
    mavsdk_interface::takeoff takeoff_srv;
    mavsdk_interface::kill kill_srv;
    mavsdk_interface::go go_srv;

public:
    InterfaceClient(ros::NodeHandle nh);
    ~InterfaceClient();

    bool call(char **argv);
};

#endif