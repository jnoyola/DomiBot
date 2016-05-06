################Initialize the Cmake Defaults#################
cmake_minimum_required(VERSION 2.6)

#Name the project
project(project)

#Set the build mode to debug by default
#SET(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_BUILD_TYPE Release)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

################Initialize the 3rdParty lib#################

#Set scl base directory
if(APPLE)
	SET(SCL_BASE_DIR ../../../../3rdparty/scl-manips-v2.git/)
else()
	SET(SCL_BASE_DIR ../../../)
endif()

###(a) Scl controller
SET(SCL_INC_DIR ${SCL_BASE_DIR}src/scl/)
SET(SCL_INC_DIR_BASE ${SCL_BASE_DIR}src/)
SET(TAO_INC_DIR ${SCL_INC_DIR}dynamics/tao/)
ADD_DEFINITIONS(-DTIXML_USE_STL)

###(b) Eigen
SET(EIGEN_INC_DIR ${SCL_BASE_DIR}3rdparty/eigen/)

###(c) Chai3D scenegraph
SET(CHAI_INC_DIR ${SCL_BASE_DIR}3rdparty/chai3d-3.0/chai3d/)
ADD_DEFINITIONS(-D_MACOSX -DMACOSX)

### (d) sUtil code
SET(SUTIL_INC_DIR ${SCL_BASE_DIR}3rdparty/sUtil/src/)

### (e) scl_tinyxml (parser)
if(APPLE)
	SET(TIXML_INC_DIR ${SCL_BASE_DIR}3rdparty/tinyxml)
else()
	SET(TIXML_INC_DIR ../../3rdparty/tinyxml)
endif()

if(APPLE)
	SET(GLEW_PATH /usr/local/Cellar/glew/1.13.0/)
endif()

################Initialize the executable#################
#Set the include directories
INCLUDE_DIRECTORIES(
	${SCL_INC_DIR}
	${SCL_INC_DIR_BASE}
	${TAO_INC_DIR}
	${EIGEN_INC_DIR}
	${CHAI_INC_DIR}
	${SUTIL_INC_DIR}
	${TIXML_INC_DIR}
	/opt/X11/include/
	${GLEW_PATH}include/
)

#Set the compilation flags
if(APPLE)
	SET(CMAKE_CXX_FLAGS "-Wall -fPIC")
else()
	SET(CMAKE_CXX_FLAGS "-Wall -fPIC -fopenmp")
endif()
SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb -O0 -pg -std=c++0x -DGRAPHICS_ON -DASSERT=assert -DDEBUG=1")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++0x -DGRAPHICS_ON -DW_THREADING_ON -DNDEBUG")

#Set all the sources required for the library
if(APPLE)
	SET(ALL_SRC project.cpp
            ${SCL_INC_DIR}/graphics/chai/CGraphicsChai.cpp 
            ${SCL_INC_DIR}/graphics/chai/ChaiGlutHandlers.cpp
            ${SCL_INC_DIR}/graphics/chai/data_structs/SGraphicsChai.cpp)
else()
	SET(TUTE_BASE_DIR ${SCL_BASE_DIR}/tutorial/DomiBot/simulation/)
	SET(ALL_SRC ${TUTE_BASE_DIR}/project.cpp
            ${SCL_INC_DIR}/graphics/chai/CGraphicsChai.cpp 
            ${SCL_INC_DIR}/graphics/chai/ChaiGlutHandlers.cpp)
endif()
	

#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_executable(project ${ALL_SRC})

###############SPECIAL CODE TO FIND AND LINK SCL's LIB DIR ######################
find_library( SCL_LIBRARY_DEBUG NAMES scl
            PATHS   ${SCL_BASE_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES debug )

find_library( SCL_LIBRARY_RELEASE NAMES scl
            PATHS   ${SCL_BASE_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES release )

SET( SCL_LIBRARY debug     ${SCL_LIBRARY_DEBUG}
              optimized ${SCL_LIBRARY_RELEASE} )

target_link_libraries(project ${SCL_LIBRARY})

###############SPECIAL CODE TO FIND AND LINK CHAI's LIB DIR ######################
find_library( CHAI_LIBRARY_DEBUG NAMES chai3d
            PATHS   ${CHAI_INC_DIR}../lib/
            PATH_SUFFIXES debug )

find_library( CHAI_LIBRARY_RELEASE NAMES chai3d
            PATHS   ${CHAI_INC_DIR}../lib/
            PATH_SUFFIXES release )

SET( CHAI_LIBRARY debug     ${CHAI_LIBRARY_DEBUG}
              optimized ${CHAI_LIBRARY_RELEASE} )

target_link_libraries(project ${CHAI_LIBRARY})


###############CODE TO FIND AND LINK REMANING LIBS ######################
FIND_LIBRARY(COCOA_LIBRARY Cocoa)
FIND_LIBRARY(OpenGL_LIBRARY OpenGL)
if(APPLE)
	target_link_libraries(project
		${GLEW_PATH}lib/libGLEW.dylib
		/opt/X11/lib/libglut.dylib
		ncurses
		dl
		${COCOA_LIBRARY}
		${OpenGL_LIBRARY}
	)
else()
	target_link_libraries(project ${CHAI_LIBRARY})
endif()
