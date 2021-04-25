#ifndef MAVSDK_UTILS_hpp
#define MAVSDK_UTILS_hpp

#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <iostream>
#include <thread>
#include <future>
#include <memory>

#define ERROR_CONSOLE_TEXT "\033[31m"     // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m"     // Restore normal console colour

// copy pasta from: https://github.com/mavlink/MAVSDK/blob/main/examples/takeoff_land/takeoff_and_land.cpp
std::shared_ptr<mavsdk::System> ConnectToDrone(mavsdk::Mavsdk &mavsdk, std::string connection_url)
{

    auto connection_result = mavsdk.add_any_connection(connection_url);

    if (connection_result != mavsdk::ConnectionResult::Success)
    {
        std::cout << ERROR_CONSOLE_TEXT << "Connection failed: " << connection_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return nullptr;
    }

    std::cout << "Waiting to discover system..." << std::endl;
    auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
    auto fut = prom.get_future();

    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot())
        {
            std::cout << "Discovered autopilot" << std::endl;

            // Unsubscribe again as we only want to find one system.
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds.
    if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::timeout)
    {
        std::cout << ERROR_CONSOLE_TEXT << "No autopilot found, exiting." << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return nullptr;
    }

    return fut.get();
}

#endif