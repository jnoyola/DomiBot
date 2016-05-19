cmake_minimum_required(VERSION 2.8.8)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# QT stuff..
FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
SET(QT_USE_QTOPENGL true)
INCLUDE(${QT_USE_FILE})
INCLUDE_DIRECTORIES(${QT_INCLUDES})

# Schunk Directory
SET(SCHUNK_DIRECTORY ../SchunkGripper/)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${SCHUNK_DIRECTORY}include/)

SET(CMAKE_CXX_FLAGS "-Wall -std=c++11")

add_executable(${PROJECT_NAME} GripperMain.cpp)

find_library( SCHUNKGRIPPER NAMES SchunkGripper
	      PATHS ${SCHUNK_DIRECTORY}build/)

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${SCHUNKGRIPPER})

target_link_libraries(${PROJECT_NAME} -lrlhal)
