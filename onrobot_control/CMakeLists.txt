
cmake_minimum_required(VERSION 2.8.3)
project(onrobot_control)
find_package(catkin REQUIRED control_msgs ur_msgs urdf)
catkin_python_setup()
catkin_package()
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})