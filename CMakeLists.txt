cmake_minimum_required(VERSION 3.0)
project(ladybug_debayer)

add_compile_options(-std=c++14 -Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
        image_transport
        nodelet
        roscpp
        sensor_msgs
        ladybug_msgs
        )

find_package(OpenCV 3 REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")
find_package(DC1394 REQUIRED)

catkin_package(CATKIN_DEPENDS
        nodelet roscpp sensor_msgs ladybug_msgs image_transport
        )

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${DC1394_INCLUDE_DIRS})

# Build debayer library
add_library(LadybugDebayer
        src/LadybugDebayer.cpp)

target_link_libraries(LadybugDebayer
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${DC1394_LIBRARIES})

add_dependencies(LadybugDebayer
        ${catkin_EXPORTED_TARGETS})

# Build nodelet
add_library(DebayerNodelet
        src/DebayerNodelet.cpp)

target_link_libraries(DebayerNodelet
        LadybugDebayer
        ${catkin_LIBRARIES})

add_dependencies(DebayerNodelet
        ${catkin_EXPORTED_TARGETS})

# Installation
install(TARGETS
        LadybugDebayer
        DebayerNodelet
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
