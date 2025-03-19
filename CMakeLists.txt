cmake_minimum_required(VERSION 3.0.2)
project(uav_landing)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    gazebo_ros
    gazebo_msgs
    image_transport
    cv_bridge
    sensor_msgs
    geometry_msgs
    mavros_msgs
    tf2
)

find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

set(aruco_INCLUDE_DIRS /usr/local/include)
set(aruco_LIBRARY_DIRS /usr/local/lib)
set(aruco_LIBRARIES /usr/local/lib/libaruco.so)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    ${aruco_LIBRARY_DIRS}
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${aruco_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_package(
    DEPENDS
        rospy
        roscpp
        gazebo_ros
        gazebo_msgs
        image_transport
        cv_bridge
        mavros_msgs
)

add_executable(aruco_estimator src/aruco_estimator.cpp)
target_link_libraries(aruco_estimator
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${OpenCV_LIBS}
    ${aruco_LIBRARIES}
)
