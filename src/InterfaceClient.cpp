#include <ros/ros.h>
#include <cstdint>
#include <iostream>
#include <mavsdk/mavsdk.h>
#include <mavsdk_interface/arm.h>
#include <mavsdk_interface/takeoff.h>
#include <chrono>
#include <thread>
#include "mavsdkUtils.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "interface_client");
    ros::NodeHandle nh;
    ros::ServiceClient arm_clt = nh.serviceClient<mavsdk_interface::arm>("mavsdk/service/arm");
    ros::ServiceClient takeoff_clt = nh.serviceClient<mavsdk_interface::takeoff>("mavsdk/service/takeoff");

    mavsdk_interface::arm arm_srv;
    mavsdk_interface::takeoff takeoff_srv;
    arm_srv.request.str1 = "go";
    takeoff_srv.request.str11 = "go";
    std::chrono::seconds timespan(5); // or whatever



    if(arm_clt.call(arm_srv)){
        ROS_INFO("GOOD JOB, DRONE ARMED");
    }
    std::this_thread::sleep_for(timespan);
    if(takeoff_clt.call(takeoff_srv)){
        ROS_INFO("GOOD JOB, DRONE SHOULD TAKING OFF");
    }
    else
    {
        ROS_ERROR("YOU DUMB BITCH");
        return 1;
    }
    return 0;
}
