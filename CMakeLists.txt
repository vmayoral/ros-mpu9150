cmake_minimum_required(VERSION 2.8.3)
project(mpu9150)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES mpu9150
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Alternatively we can use the CMakeLists.txt of the subdirs
#add_subdirectory(src/linux-mpu9150)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  src/linux-mpu9150
  src/linux-mpu9150/eMPL
  src/linux-mpu9150/glue
  src/linux-mpu9150/mpu9150
)

#link_directories()


add_executable(random src/random.cpp)
target_link_libraries(random ${catkin_LIBRARIES} )

add_executable(mpu9150_node src/mpu9150_node.cpp)
add_library(quaternion SHARED src/linux-mpu9150/mpu9150/quaternion.c)
add_library(vector3d SHARED src/linux-mpu9150/mpu9150/vector3d.c)
add_library(mpu9150 SHARED src/linux-mpu9150/mpu9150/mpu9150.c)
add_library(inv_mpu SHARED src/linux-mpu9150/eMPL/inv_mpu.c)
add_library(inv_mpu_dmp_motion_driver SHARED src/linux-mpu9150/eMPL/inv_mpu_dmp_motion_driver.c)
add_library(linux_glue SHARED src/linux-mpu9150/glue/linux_glue.c)

target_link_libraries(mpu9150_node ${catkin_LIBRARIES} inv_mpu inv_mpu_dmp_motion_driver linux_glue mpu9150 quaternion vector3d)
#target_link_libraries(mpu9150_node ${catkin_LIBRARIES} )

