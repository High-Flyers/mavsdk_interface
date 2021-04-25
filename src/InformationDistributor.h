#ifndef InformationDistributor_h
#define InformationDistributor_h

#include <ros/ros.h>
#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <thread>
#include <future>
#include <memory>
#include <mavsdk_interface/gpsPos.h>
#include <mavsdk_interface/battery.h>
#include <nav_msgs/Odometry.h>
#include <mavsdk_interface/velocityNedPos.h>
#include <mavsdk_interface/flightMode.h>
#include <limits>
#include <math.h>

class InformationDistributor
{
public:
    InformationDistributor();
    ~InformationDistributor();

    void subcribePosition(ros::Publisher &posGPS_pub, std::shared_ptr<mavsdk::Telemetry> telemetry);

    void subcribeBattery(ros::Publisher &battery_pub, std::shared_ptr<mavsdk::Telemetry> telemetry);

    void subcribeOdometry(ros::Publisher &odometry_pub, std::shared_ptr<mavsdk::Telemetry> telemetry);

    void subcribePositionVelocityNed(ros::Publisher &velNedPos_pub, std::shared_ptr<mavsdk::Telemetry> telemetry);

    void subcribeFlightMode(ros::Publisher &flightMode_pub, std::shared_ptr<mavsdk::Telemetry> telemetry);
};
#endif