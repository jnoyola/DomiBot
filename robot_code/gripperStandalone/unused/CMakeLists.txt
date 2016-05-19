cmake_minimum_required(VERSION 2.8.8)
project(gripperStandalone)

find_package(catkin REQUIRED COMPONENTS MathLib StdTools RobotLib usefulLibraries bullet SchunkGripper roscpp message_generation)

catkin_package()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# QT stuff..
FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
SET(QT_USE_QTOPENGL true)
INCLUDE(${QT_USE_FILE})
INCLUDE_DIRECTORIES(${QT_INCLUDES})

#QT4_WRAP_UI(UI_SRCS mainwindow.ui)
#QT4_WRAP_CPP(MOC_SRCS mainwindow.h)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${catkin_INCLUDE_DIRS})

#add_executable(${PROJECT_NAME} GripperMain.cpp mainwindow.cpp ${MOC_SRCS} ${UI_SRCS})
add_executable(${PROJECT_NAME} GripperMain.cpp)

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} -lrlhal)


# remove strip the path such that it give the path to RobotToolKit
#string(REPLACE "/packages/core/${PROJECT_NAME}" "" RobotToolKit_PATH ${PROJECT_SOURCE_DIR})

#IF(NOT(EXISTS "${RobotToolKit_PATH}/bin"))
#	MAKE_DIRECTORY(${RobotToolKit_PATH}/bin)
#	message("WARNING: The folder ${RobotToolKit_PATH}/bin does not exist, so I created it.")
#ENDIF(NOT(EXISTS "${RobotToolKit_PATH}/bin"))   

#ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ln -sf ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${PROJECT_NAME} "${RobotToolKit_PATH}/bin/${PROJECT_NAME}")
