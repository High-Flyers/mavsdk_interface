cmake_minimum_required(VERSION 3.0.2)
project(mavsdk_interface)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  message_generation
  actionlib_msgs
  actionlib
)

add_action_files(
  DIRECTORY action
  FILES fly.action
)

add_message_files(
  FILES
  gpsPos.msg
  battery.msg
  velocityNedPos.msg
  flightMode.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(MAVSDK REQUIRED)

add_executable(node src/interface.cpp)
add_executable(fly src/flyClient.cpp)
add_dependencies(node ${catkin_EXPORTED_TARGETS})
add_dependencies(fly ${catkin_EXPORTED_TARGETS})
target_link_libraries(node ${catkin_LIBRARIES} 
                      MAVSDK::mavsdk  
                      MAVSDK::mavsdk_telemetry
                      MAVSDK::mavsdk_action)
                      
target_link_libraries(fly ${catkin_LIBRARIES} 
                      MAVSDK::mavsdk  
                      MAVSDK::mavsdk_telemetry
                      MAVSDK::mavsdk_action)
