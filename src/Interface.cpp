#include <ros/ros.h>
#include <cstdint>
#include <iostream>
#include "Interface.h"


#define ROS_RATE 60



Interface::Interface(ros::NodeHandle &nh, std::string udp) 
{
    posGPS_pub = nh.advertise<mavsdk_interface::gpsPos>("t_mavsdk_node/mavsdk/gpsPos", 10);
    battery_pub  = nh.advertise<mavsdk_interface::battery>("t_mavsdk_node/mavsdk/battery", 10);
    odo_pub = nh.advertise<nav_msgs::Odometry>("t_mavsdk_node/mavsdk/odometry", 10);
    velNedPos_pub = nh.advertise<mavsdk_interface::velocityNedPos>("t_mavsdk_node/mavsdk/velocityNedPos", 10);
    flightMode_pub = nh.advertise<mavsdk_interface::flightMode>("t_mavsdk_node/mavsdk/flightMode", 10);

    arm_srv = nh.advertiseService("t_mavsdk_node/mavsdk/service/arm", &Interface::isArmed, this);
    takeoff_srv = nh.advertiseService("t_mavsdk_node/mavsdk/service/takeoff", &Interface::takeoff, this);
    kill_srv = nh.advertiseService("t_mavsdk_node/mavsdk/service/kill", &Interface::kill, this);
    go_srv = nh.advertiseService("t_mavsdk_node/mavsdk/service/go", &Interface::go, this);
    stopOffboard_srv = nh.advertiseService("t_mavsdk_node/mavsdk/service/stopOffboard", &Interface::stopOffboard, this);

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
    this->offboard->set_velocity_body({req.x, req.y, req.z, req.yaw});
    

    // Start offboard mode if not already set.
    if(isOffboard == false)
    {
        mavsdk::Offboard::Result offboard_result = this->offboard->start();
        if (offboard_result != mavsdk::Offboard::Result::Success) {
                std::cerr << "Offboard::start() failed: " << offboard_result << '\n';
                res.str2 = "Error: failed to start offboard";
                return 0;
        }
        isOffboard = true;
    }
    res.str2 = "succes!";
    return 1;
}

bool Interface::stopOffboard(mavsdk_interface::stopOffboard::Request &req, mavsdk_interface::stopOffboard::Response &res)
{
    if(req.confirmation == false)
    {
        res.result = false;
        return 1;
    }

    if(isOffboard == false)
    {
        res.result = true;
        return 1;
    }

    auto result = this->offboard->stop();
    if(result == mavsdk::Offboard::Result::Success)
    {
        res.result = true;
        isOffboard = false;
        return 1;
    }

    res.result = false;
    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "t_mavsdk_node");
    ros::NodeHandle nh;

    // default port
    std::string port = "udp://:14540";
    nh.getParam("mavlink_port", port);
    
    if(port == "")
    {
        std::cerr << "SPECIFY THE BIND ADDRESS\n";
        return 1;
    }
    ROS_INFO("Using mavlink_port: %s", port.c_str());

    auto interface = new Interface(nh, port);
  
    ros::spin();
    return 0;
}

    