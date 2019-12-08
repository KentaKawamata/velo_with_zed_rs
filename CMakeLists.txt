cmake_minimum_required(VERSION 2.8.3)
project(velo_with_zed_rs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2
  nav_msgs
  genmsg
  actionlib
  actionlib_msgs
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# アクションメッセージの定義
#####################################
add_action_files(
    DIRECTORY action
    FILES rotation.action mount.action zed.action
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)
#####################################

catkin_package(
    INCLUDE_DIRS
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS  roscpp
                    rospy
                    geometry_msgs
                    sensor_msgs
                    std_msgs
                    pcl_conversions
                    pcl_ros
                    tf2_ros 
                    tf2
                    nav_msgs
                    actionlib
                    actionlib_msgs
    DEPENDS
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   ${PCL_INCLUDE_DIRS}
   ${Eigen3_INCLUDE_DIRS}
)


#####################################
add_executable(quaternion_mount
   src/quaternion_mount.cpp
   include/quaternion_mount.h
)

add_dependencies(quaternion_mount
   ${catkin_EXPORTED_TARGETS}
   ${mount_EXPORTED_TARGETS}
)

target_link_libraries(quaternion_mount
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Eigen3_LIBRARIES}
)
#####################################


#####################################
add_executable(odom_zed
   src/odom_zed.cpp
   include/odom_zed.h
)

add_dependencies(odom_zed
   ${catkin_EXPORTED_TARGETS}
   ${mount_EXPORTED_TARGETS}
)

target_link_libraries(odom_zed
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Eigen3_LIBRARIES}
)
#####################################
