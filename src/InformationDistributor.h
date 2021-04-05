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

class InformationDistributor {
public:
    InformationDistributor() {};
    ~InformationDistributor() {};

    void subcribePosition(ros::Publisher& posGPS_pub, mavsdk::Telemetry & telemetry){
        telemetry.subscribe_position([&](mavsdk::Telemetry::Position pos){
            mavsdk_interface::gpsPos msg;
            msg.latitude_deg = pos.latitude_deg;
            msg.longitude_deg = pos.longitude_deg;
            msg.absolute_altidue_m = pos.absolute_altitude_m;
            msg.relative_altitude_m = pos.relative_altitude_m;
            posGPS_pub.publish(msg);
        });
    }

    void subcribeBattery(ros::Publisher& battery_pub, mavsdk::Telemetry & telemetry){
        telemetry.subscribe_battery([&](mavsdk::Telemetry::Battery batt){
            mavsdk_interface::battery msg;
            msg.voltage_v = batt.voltage_v;
            msg.remaining_percent = batt.remaining_percent;
            battery_pub.publish(msg);
        });
    }

    void subcribeOdometry(ros::Publisher& odometry_pub, mavsdk::Telemetry & telemetry){
        telemetry.subscribe_odometry([&](mavsdk::Telemetry::Odometry odo){
            nav_msgs::Odometry msg;
            msg.pose.pose.orientation.w = (double)odo.q.w;
            msg.pose.pose.orientation.x = (double)odo.q.x;
            msg.pose.pose.orientation.y = (double)odo.q.y;
            msg.pose.pose.orientation.z = (double)odo.q.z;
            msg.pose.pose.position.x = (double)odo.position_body.x_m;
            msg.pose.pose.position.y = (double)odo.position_body.y_m;
            msg.pose.pose.position.z = (double)odo.position_body.z_m;
            for(int i = 0; i < 36; i++){
                msg.pose.covariance[i] = (double)odo.pose_covariance.covariance_matrix[i];
                msg.twist.covariance[i] = (double)odo.velocity_covariance.covariance_matrix[i];
            }
            msg.twist.twist.angular.x = (double)odo.angular_velocity_body.roll_rad_s;
            msg.twist.twist.angular.y = (double)odo.angular_velocity_body.pitch_rad_s;
            msg.twist.twist.angular.z = (double)odo.angular_velocity_body.yaw_rad_s;
            msg.twist.twist.linear.x = (double)odo.velocity_body.x_m_s;
            msg.twist.twist.linear.y = (double)odo.velocity_body.y_m_s;
            msg.twist.twist.linear.z = (double)odo.velocity_body.z_m_s;
            odometry_pub.publish(msg);

        });
    }

    void subcribePositionVelocityNed(ros::Publisher& velNedPos_pub, mavsdk::Telemetry & telemetry){
        telemetry.subscribe_position_velocity_ned([&](mavsdk::Telemetry::PositionVelocityNed velNedPos){
            mavsdk_interface::velocityNedPos msg;
            msg.down_m = velNedPos.position.down_m;
            msg.east_m = velNedPos.position.east_m;
            msg.north_m = velNedPos.position.north_m;
            msg.down_m_s = velNedPos.velocity.down_m_s;
            msg.east_m_s = velNedPos.velocity.east_m_s;
            msg.north_m_s = velNedPos.velocity.north_m_s;
            velNedPos_pub.publish(msg);
        });
    }

    void subcribeFlightMode(ros::Publisher& flightMode_pub, mavsdk::Telemetry & telemetry){
        telemetry.subscribe_flight_mode([&](mavsdk::Telemetry::FlightMode flightMode){
            mavsdk_interface::flightMode msg;
            switch (flightMode)
            {
            case mavsdk::Telemetry::FlightMode::Unknown:
                msg.flight_mode = "Unknown";
                break;
            case mavsdk::Telemetry::FlightMode::Ready:
                msg.flight_mode = "Ready";
                break;
            case mavsdk::Telemetry::FlightMode::Takeoff:
                msg.flight_mode = "Takeoff";
                break;
            case mavsdk::Telemetry::FlightMode::Hold:
                msg.flight_mode = "Hold";
                break;
            case mavsdk::Telemetry::FlightMode::Mission:
                msg.flight_mode = "Mission";
                break;
            case mavsdk::Telemetry::FlightMode::ReturnToLaunch:
                msg.flight_mode = "ReturnToLaunch";
                break;
            case mavsdk::Telemetry::FlightMode::Land:
                msg.flight_mode = "Land";
                break;
            case mavsdk::Telemetry::FlightMode::Offboard:
                msg.flight_mode = "Offboard";
                break;
            case mavsdk::Telemetry::FlightMode::FollowMe:
                msg.flight_mode = "FollowMe";
                break;
            case mavsdk::Telemetry::FlightMode::Manual:
                msg.flight_mode = "Manual";
                break;
            case mavsdk::Telemetry::FlightMode::Altctl:
                msg.flight_mode = "Altctl";
                break;
            case mavsdk::Telemetry::FlightMode::Posctl:
                msg.flight_mode = "Posctl";
                break;
            case mavsdk::Telemetry::FlightMode::Acro:
                msg.flight_mode = "Acro";
                break;
            case mavsdk::Telemetry::FlightMode::Stabilized:
                msg.flight_mode = "Stabilized";
                break;
            case mavsdk::Telemetry::FlightMode::Rattitude:
                msg.flight_mode = "Rattitude";
                break;
            default:
                msg.flight_mode = "WTF?";
                break;
            }
            flightMode_pub.publish(msg);
        });
    }
};
