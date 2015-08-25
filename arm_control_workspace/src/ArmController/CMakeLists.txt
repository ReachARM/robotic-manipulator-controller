cmake_minimum_required(VERSION 2.8.3)
project(ArmController)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_service_files(
  FILES
  MoveRelativeTool.srv
  MoveAbsoluteMotor.srv
  MoveIncrementMotor.srv
  MoveBase.srv
  GetArmStatus.srv
  GetMotorAngle.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ArmController
   CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

file(GLOB ROS src/ros/*.h src/ros/*.cpp)
file(GLOB MOTOR src/motor/*.h src/motor/*.cpp)
file(GLOB CONTROLLER src/*.h src/*.cpp)

add_executable(ArmController_node src/ArmController_node.cpp ${CONTROLLER} ${MOTOR} ${ROS})

add_dependencies(ArmController_node ArmController_generate_messages_cpp)

target_link_libraries(ArmController_node
   ${catkin_LIBRARIES}
   dxl
)
