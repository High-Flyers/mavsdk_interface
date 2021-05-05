#include <ros/ros.h>
#include <cstdint>
#include <iostream>
#include "Interface.h"


#define ROS_RATE 30



Interface::Interface(ros::NodeHandle &nh, std::string udp) 
{
    posGPS_pub = nh.advertise<mavsdk_interface::gpsPos>("mavsdk/gpsPos", 100);
    battery_pub  = nh.advertise<mavsdk_interface::battery>("mavsdk/battery", 100);
    odo_pub = nh.advertise<nav_msgs::Odometry>("mavsdk/odometry", 100);
    velNedPos_pub = nh.advertise<mavsdk_interface::velocityNedPos>("mavsdk/velocityNedPos", 100);
    flightMode_pub = nh.advertise<mavsdk_interface::flightMode>("mavsdk/flightMode", 100);

    arm_srv = nh.advertiseService("mavsdk/service/arm", &Interface::isArmed, this);
    takeoff_srv = nh.advertiseService("mavsdk/service/takeoff", &Interface::takeoff, this);
    kill_srv = nh.advertiseService("mavsdk/service/kill", &Interface::kill, this);
    go_srv = nh.advertiseService("mavsdk/service/go", &Interface::go, this);

    auto system = ConnectToDrone(mavsdk, udp);

    telemetry = std::make_shared<mavsdk::Telemetry>(system);
    action = std::make_shared<mavsdk::Action>(system);
    offboard = std::make_shared<mavsdk::Offboard>(system);

    distibutor.subcribePosition(posGPS_pub, telemetry);
    distibutor.subcribeBattery(battery_pub, telemetry);
    distibutor.subcribeOdometry(odo_pub, telemetry);
    distibutor.subcribePositionVelocityNed(velNedPos_pub, telemetry);
    distibutor.subcribeFlightMode(flightMode_pub, telemetry);
}

Interface::~Interface()
{
}

bool Interface::isArmed(mavsdk_interface::arm::Request &req, mavsdk_interface::arm::Response &res)
{
    const mavsdk::Action::Result arm_result = this->action->arm();
    if(arm_result != mavsdk::Action::Result::Success){
        return 0;
    }
    return 1;
}

bool Interface::takeoff(mavsdk_interface::takeoff::Request &req, mavsdk_interface::takeoff::Response &res)
{
    const mavsdk::Action::Result takeoff_result = this->action->takeoff();
    if(takeoff_result != mavsdk::Action::Result::Success){
        return 0;
    }
    return 1;
}


bool Interface::kill(mavsdk_interface::kill::Request &req, mavsdk_interface::kill::Response &res)
{
    const mavsdk::Action::Result kill_result = this->action->kill();
    if(kill_result != mavsdk::Action::Result::Success){
        return 0;
    }
    return 1;
}


bool Interface::go(mavsdk_interface::go::Request &req, mavsdk_interface::go::Response &res)
{
    
    this->offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});

    // Start offboard mode.
    mavsdk::Offboard::Result offboard_result = this->offboard->start();
    if (offboard_result != mavsdk::Offboard::Result::Success) {
            std::cerr << "Offboard::start() failed: " << offboard_result << '\n';
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "t_mavsdk_node");
    ros::NodeHandle nh("~");

    // default port
    std::string port = "udp://:14540";
    nh.getParam("mavlink_port", port);
    
    if(port == "")
    {
        std::cerr << "SPECIFY THE BIND ADDRESS\n";
        return 1;
    }
    ROS_INFO("Using mavlink_port: %s", port.c_str());

    Interface Interface(nh, port);
  
    ros::Rate loop_rate(ROS_RATE);
    
    
    ros::spin();
    return 0;
}

    