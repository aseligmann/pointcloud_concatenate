cmake_minimum_required(VERSION 3.0.2)
### Project name
project(pointcloud_concatenate)

## Use C++14
set(CMAKE_CXX_STANDARD 14)



### Find dependencies
find_package(catkin REQUIRED COMPONENTS # Must be same as CATKIN_DEPENDS
        roscpp
        pcl_ros
        tf2
        tf2_ros
        sensor_msgs
)
find_package(PCL REQUIRED)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)



############
## catkin ##
############

### Define package
catkin_package(
        INCLUDE_DIRS 
                include
                ${EIGEN3_INCLUDE_DIR} # This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
        LIBRARIES 
                ${PROJECT_NAME}
        CATKIN_DEPENDS # Must be same as CATKIN_DEPENDS
                roscpp
                pcl_ros
                tf2
                tf2_ros
                sensor_msgs
)



###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
        include
        ${THIS_PACKAGE_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR} # Set manually because Eigen sets a non standard INCLUDE DIR
        ${Boost_INCLUDE_DIRS} # Set because Boost is an internal dependency, not transitive.
)

# Declare a cpp library
add_library(${PROJECT_NAME} 
        src/${PROJECT_NAME}.cpp
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

# Declare cpp executables
add_executable(${PROJECT_NAME}_node 
        src/${PROJECT_NAME}_node.cpp
)
target_compile_features(${PROJECT_NAME}_node INTERFACE cxx_std_11)
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
)



#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
        FILES launch/concat.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
