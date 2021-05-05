cmake_minimum_required(VERSION 3.0.2)
project(mavsdk_interface)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  tf
  message_generation
)

add_message_files(
  FILES
  gpsPos.msg
  battery.msg
  velocityNedPos.msg
  flightMode.msg
)

add_service_files(
  FILES
  arm.srv
  takeoff.srv
  kill.srv
  go.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(MAVSDK REQUIRED)

add_executable(node src/Interface.cpp
                src/InformationDistributor.cpp)
add_executable(node_base_link_tf src/base_link_tf.cpp)
add_executable(node_client src/InterfaceClient.cpp)


add_dependencies(node ${catkin_EXPORTED_TARGETS})

add_dependencies(node_client ${catkin_EXPORTED_TARGETS})


target_link_libraries(node ${catkin_LIBRARIES} 
                      MAVSDK::mavsdk  
                      MAVSDK::mavsdk_telemetry
                      MAVSDK::mavsdk_action
                      MAVSDK::mavsdk_offboard)


target_link_libraries(node_client ${catkin_LIBRARIES}
                      MAVSDK::mavsdk)

target_link_libraries(node_base_link_tf ${catkin_LIBRARIES} )
