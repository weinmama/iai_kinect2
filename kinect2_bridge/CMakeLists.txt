cmake_minimum_required(VERSION 3.18)
project(kinect2_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(DEPENDENCIES
        rclcpp
        rclcpp_components
        rcutils
        kinect2_bridge_component)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(kinect2_bridge_component REQUIRED)

###########
## Build ##
###########

add_definitions(-DK2_CALIB_PATH="${PROJECT_SOURCE_DIR}/data/")

include_directories(include ${kinect2_bridge_component_INCLUDE_DIRS})

add_executable(kinect2_bridge src/kinect2_bridge.cpp)

ament_target_dependencies(kinect2_bridge ${DEPENDENCIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

# Install PARAMS files
#install(DIRECTORY
#        config
#        DESTINATION share/${PROJECT_NAME}
#        )
#
## Install URDF files
#install(DIRECTORY
#        urdf
#        DESTINATION share/${PROJECT_NAME}
#        )

# Install LAUNCH files
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
        )

#############
## Testing ##
#############

ament_package()
