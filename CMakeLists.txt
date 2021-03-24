cmake_minimum_required(VERSION 3.0.2)
project(mavsdk_interface)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  gpsPos.msg
  battery.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(MAVSDK REQUIRED)

add_executable(node src/interface.cpp)
add_dependencies(node ${catkin_EXPORTED_TARGETS})
target_link_libraries(node ${catkin_LIBRARIES} MAVSDK::mavsdk  MAVSDK::mavsdk_telemetry)
