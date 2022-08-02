cmake_minimum_required(VERSION 2.8.3)
project(ros_system_monitor)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  diagnostic_msgs
  er_robot_status_msgs
)

catkin_package()


install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS bin/cpu_monitor.py bin/hdd_monitor.py bin/mem_monitor.py  bin/net_monitor.py bin/ntp_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
