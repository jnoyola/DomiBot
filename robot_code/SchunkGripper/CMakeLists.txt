cmake_minimum_required(VERSION 2.8.3)
project(SchunkGripper)

#common commands for building c++ executables and libraries
FILE(GLOB theSourceFiles src/*.c*)
FILE(GLOB theHeaderFiles include/*.h)
include_directories(./include)
add_library(${PROJECT_NAME} ${theSourceFiles} )

FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
SET(QT_USE_QTOPENGL true)
INCLUDE(${QT_USE_FILE})
INCLUDE_DIRECTORIES(${QT_INCLUDES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

target_link_libraries(${PROJECT_NAME} -lrlhal)
