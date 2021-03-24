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

    ros::Rate loop_rate(ROS_RATE);

    // MAVSDK SETUP --------
    mavsdk::Mavsdk mavsdk;
    mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);
    if(connection_result != mavsdk::ConnectionResult::Success) {
        std::cerr << "failed to connect\n";
        return 1;
    }
        
    std::atomic<size_t> num_systems_discovered{0};

    std::cout << "Waiting to discover systems...\n";
    mavsdk.subscribe_on_new_system([&]() {
        const auto systems = mavsdk.systems();
        
        if (systems.size() > num_systems_discovered) {
            std::cout << "Discovered system" << std::endl;
            num_systems_discovered = systems.size();
        }
    });


    for(int i = 0; i < ROS_RATE * 2; i++)
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
    int correct_system = 0;

    if (num_systems_discovered != 1) {
        if(num_systems_discovered > 1)
        {
            std::cerr << "Found more than one drone! Num: " << num_systems_discovered << std::endl;
            std::cerr << "Trying to filter out UUID 255" << std::endl;
            for(int i = 0; i < num_systems_discovered; i++)
            {
                if(mavsdk.systems().at(i)->get_uuid() != 255)
                {
                    correct_system = i;
                    std::cerr <<  "Success, found different uuid" << std::endl;
                    goto end_connect;
                }
            }
            std::cerr << "Failed to filter 255" << std::endl;
            return 0;
        }
        else
        {
            std::cerr << "Drone has not been found" << std::endl;
            return 0;
        }
    }
    end_connect:

    auto system = mavsdk.systems().at(0);
    auto telemetry = mavsdk::Telemetry{system};

    // END MAVSDK SETUP --------

    

    telemetry.subscribe_position([&](mavsdk::Telemetry::Position pos){
        mavsdk_interface::gpsPos msg;
        msg.latitude_deg = pos.latitude_deg;
        msg.longitude_deg = pos.longitude_deg;
        msg.absolute_altidue_m = pos.absolute_altitude_m;
        msg.relative_altitude_m = pos.relative_altitude_m;
        posGPS_pub.publish(msg);
    });

    telemetry.subscribe_battery([&](mavsdk::Telemetry::Battery batt){
        mavsdk_interface::battery msg;
        msg.voltage_v = batt.voltage_v;
        msg.remaining_percent = batt.remaining_percent;
        battery_pub.publish(msg);
    });
    ros::spin();

    return 0;
}

