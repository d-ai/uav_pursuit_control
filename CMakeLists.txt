cmake_minimum_required(VERSION 3.0.2)
project(anafi_ros1)




find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(catkin REQUIRED COMPONENTS rospy)
find_package(catkin REQUIRED COMPONENTS std_msgs)
find_package(catkin REQUIRED COMPONENTS tf)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_generation REQUIRED)
find_package(genmsg REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(roslaunch REQUIRED)


message( STATUS "OpenCV  Di: " ${OpenCV_INCLUDE_DIRS} )

# Message Declaration #
add_message_files(FILES spData.msg traj.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES anafi
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(include  ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})



add_executable(uav_pid src/af_pid.cpp)
target_link_libraries(uav_pid ${catkin_LIBRARIES})
target_link_libraries(uav_pid ${cv_bridge_LIBRARIES})
target_link_libraries(uav_pid ${OpenCV_LIBS})
target_link_libraries(uav_pid ${tf_LIBRARIES})
target_link_libraries(uav_pid ${Eigen_LIBRARIES})


add_executable(uav_speed src/af_speed.cpp)
target_link_libraries(uav_speed ${catkin_LIBRARIES})
target_link_libraries(uav_speed ${cv_bridge_LIBRARIES})
target_link_libraries(uav_speed ${OpenCV_LIBS})
target_link_libraries(uav_speed ${tf_LIBRARIES})
target_link_libraries(uav_speed ${Eigen_LIBRARIES})
