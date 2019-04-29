cmake_minimum_required(VERSION 2.8.3)
project(monodepth_localization)

set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC")

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE RelWithDebInfo)
endif ()

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "./cmake")
file (GLOB MODULES "${CMAKE_MODULE_PATH}/*.cmake")
foreach (MODULE ${MODULES})
  include (${MODULE})
endforeach ()

find_package(catkin REQUIRED COMPONENTS 
roscpp rospy std_msgs sensor_msgs
image_transport
cv_bridge
pcl_conversions
pcl_msgs
pcl_ros
tf
tf_conversions
eigen_conversions
irp_slam_msgs
irp_sen_msgs
lib_irp_common
lib_irp_math)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(libLAS REQUIRED)
find_package(Ceres REQUIRED)
libhandler_isam ()
#find_package(G2O REQUIRED)
#find_package(PCL 1.7 REQUIRED)

## Declare a catkin package
catkin_package(DEPENDS OpenCV Eigen3 libLAS isam
               INCLUDE_DIRS include
               LIBRARIES monodepth_localization
               CATKIN_DEPENDS roscpp  
               std_msgs sensor_msgs image_transport geometry_msgs
               irp_slam_msgs
               irp_sen_msgs
               lib_irp_common
               lib_irp_math
               cv_bridge
               pcl_ros pcl_conversions pcl_msgs tf tf_conversions eigen_conversions)

# Also add some useful compiler flag
set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} ${SSE_FLAGS} -march=native  -std=c++0x"
) 

###########
## Build ##
###########

set(CMAKE_BUILD_TYPE Release)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${libLAS_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
    ${ISAM_INCLUDE_DIRS}
#    SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS}
)
#include_directories(SYSTEM BEFORE ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})

set (SRC
  src/main.cpp
  src/CamLocalization.cpp
  src/MapPublisher.cpp
  src/isamClient.cpp
)

#link_directories(${G2O_LIBRARY_DIRS})

add_executable(monodepth_localization ${SRC})
target_link_libraries(monodepth_localization 
                      ${catkin_LIBRARIES} 
                      ${OpenCV_LIBS}
                      ${libLAS_LIBRARIES}
                      ${CERES_LIBRARIES}
                      ${ISAM_LIBRARIES}
#                      ${G2O_LIBRARIES}
                      ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so)

add_dependencies(monodepth_localization ${catkin_EXPORTED_TARGETS})
