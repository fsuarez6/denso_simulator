cmake_minimum_required(VERSION 2.8.3)
project(denso_gazebo)

find_package(catkin REQUIRED COMPONENTS
  denso_description
  effort_controllers
  gazebo_ros
  gazebo_ros_control
  gripper_action_controller
  joint_state_controller
  joint_trajectory_controller
  robot_state_publisher
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
