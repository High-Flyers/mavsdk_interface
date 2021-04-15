#include <ros/ros.h>
#include <cstdint>
#include <iostream>
#include <mavsdk/mavsdk.h>
#include <mavsdk_interface/arm.h>
#include <mavsdk_interface/takeoff.h>
#include <mavsdk_interface/kill.h>
#include <chrono>
#include <thread>
#include "mavsdkUtils.hpp"

class InterfaceClient
{
private:
    ros::ServiceClient arm_clt;
    ros::ServiceClient takeoff_clt;
    ros::ServiceClient kill_clt;

    mavsdk_interface::arm arm_srv;
    mavsdk_interface::takeoff takeoff_srv;
    mavsdk_interface::kill kill_srv;
    
public:
    InterfaceClient(ros::NodeHandle nh);
    ~InterfaceClient();

    bool call(std::string arg);
};

InterfaceClient::InterfaceClient(ros::NodeHandle nh)
{
    arm_clt = nh.serviceClient<mavsdk_interface::arm>("mavsdk/service/arm");
    takeoff_clt = nh.serviceClient<mavsdk_interface::takeoff>("mavsdk/service/takeoff");
    kill_clt = nh.serviceClient<mavsdk_interface::kill>("mavsdk/service/kill");
}

InterfaceClient::~InterfaceClient()
{
}

bool InterfaceClient::call(std::string arg){
    if(arg == "arm"){
        arm_srv.request.str1 = "go";
        if(arm_clt.call(arm_srv)){
            ROS_INFO("GOOD JOB, DRONE ARMED");
            return 0;
        }
    }
    if(arg == "takeoff"){
        takeoff_srv.request.str11 = "go";
        if(takeoff_clt.call(takeoff_srv)){
            ROS_INFO("GOOD JOB, DRONE SHOULD TAKING OFF");
            return 0;
        }
    }
    if(arg == "kill"){
        kill_srv.request.str1 = "Do it Anakin, KILL HIM NOW";
        if(kill_clt.call(kill_srv)){
            ROS_INFO("GOOD JOB, THE DARK SIDE IS NEAR");
            return 0;
        }
    }
    return 1;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "interface_client");
    ros::NodeHandle nh;
    InterfaceClient client(nh);
    ROS_INFO(argv[1]);
    if(client.call(argv[1]))
    {
        ROS_ERROR("YOU DUMB BITCH");
        return 1;
    }
    return 0;
}
