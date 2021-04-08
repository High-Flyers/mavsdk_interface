#include <mavsdk_interface/flyAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include "mavsdkUtils.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fly_client");

    mavsdk::Mavsdk mavsdk;

    auto system = ConnectToDrone(mavsdk, argv[1]);
    if(system == nullptr)
        return 1;

    
    actionlib::SimpleActionClient<mavsdk_interface::flyAction> flyClient("mavsdk/fly", true);
    flyClient.waitForServer();
    mavsdk_interface::flyGoal goal;
    auto action = mavsdk::Action{system};

    const mavsdk::Action::Result arm_result = action.arm();
    const mavsdk::Action::Result takeoff_result = action.takeoff();
    flyClient.sendGoal(goal);
    flyClient.waitForResult(ros::Duration(5.0));
    if(flyClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        std::cout << "OK" << std::endl;
    return 0;
}