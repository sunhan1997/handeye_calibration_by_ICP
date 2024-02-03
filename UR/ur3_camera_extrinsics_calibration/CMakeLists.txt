cmake_minimum_required(VERSION 3.0.2)
project(ur3_camera_extrinsics_calibration)

find_package(catkin REQUIRED)

catkin_package()

install(FILES
    launch/ur3_calibrate.launch
    launch/ur3_check_calibration.launch
    launch/ur3_config.rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
