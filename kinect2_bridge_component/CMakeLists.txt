cmake_minimum_required(VERSION 3.18)
project(kinect2_bridge_component CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBT_USE_DOUBLE_PRECISION -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body -Wlogical-op")

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF (COMPILER_SUPPORTS_CXX11)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF (COMPILER_SUPPORTS_CXX0X)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE ()
    MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF ()

set(DEPENDENCIES
        rclcpp
        rclcpp_components
        rcutils
        geometry_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        image_transport
        cv_bridge
        std_msgs
        geometry_msgs
        sensor_msgs
        kinect2_registration)

find_package(ament_cmake REQUIRED)

find_package(freenect2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs)
find_package(std_msgs)
find_package(sensor_msgs)
find_package(cv_bridge)
find_package(image_transport REQUIRED)
find_package(image_transport_plugins REQUIRED)
find_package(kinect2_registration REQUIRED)

###########
## Build ##
###########

include_directories(include
        ${OpenCV_INCLUDE_DIRS}
        ${freenect2_INCLUDE_DIRS}
        ${kinect2_registration_INCLUDE_DIRS}
        )

if (DEPTH_REG_OPENCL)
    add_definitions(-DDEPTH_REG_OPENCL)
endif ()
if (DEPTH_REG_CPU)
    add_definitions(-DDEPTH_REG_CPU)
endif ()

add_definitions(-DK2_CALIB_PATH="${PROJECT_SOURCE_DIR}/data/")

##### define Target Library ####

add_library(kinect2_bridge_component SHARED src/kinect2_bridge_component.cpp)
target_link_libraries(kinect2_bridge_component
        ${OpenCV_LIBRARIES}
        ${freenect2_LIBRARY}
        )

ament_target_dependencies(kinect2_bridge_component ${DEPENDENCIES})

rclcpp_components_register_nodes(kinect2_bridge_component "composition::Kinect2Bridge")
set(node_plugins "${node_plugins}composition::Kinect2Bridge;$<TARGET_FILE:kinect2_bridge_component>\n")

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )

#############
## Testing ##
#############

#############
## Exports ##
#############
ament_export_include_directories(include)
ament_export_libraries(kinect2_bridge_component)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
