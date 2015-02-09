# Copyright: 2013 iCub Facility, Istituto Italiano di Tecnologia
# Author: Nawid Jamali
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

#
# The project root CMakeLists.
#

# ###########################################################################
# Project details
# ###########################################################################
cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME tacman-sensorCharacterisation)
project(${PROJECTNAME})

set(FORCE_DIMENSION_HOME "C:/Program\ Files/Force\ Dimension/sdk-3.5.2")

# ###########################################################################
# The included source code
# ###########################################################################
set(HEADERS src/omegaATIThread.h   )
set(SOURCES src/main.cpp src/omegaATIThread.cpp)

# ###########################################################################	
# Project Version
# ###########################################################################

set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 1)
 
set(PROJECT_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)      # Set build path
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)         # Set lib path

set(CMAKE_BUILD_TYPE Debug)

MESSAGE(STATUS "Compiling in ${CMAKE_BUILD_TYPE} mode.")
if(CMAKE_BUILD_TYPE MATCHES Debug)
    MESSAGE(STATUS "Adding debug flags to compiler.")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
else(CMAKE_BUILD_TYPE MATCHES Debug)
    MESSAGE(STATUS "Adding release flags to compiler.")
    set(GCC_DEBUG_COMPILE_FLAGS, "-g")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall ${GCC_DEBUG_COMPILE_FLAGS}")
endif(CMAKE_BUILD_TYPE MATCHES Debug)

# ###########################################################################
# iCub and YARP Includes
# ###########################################################################
# Find packages
find_package(YARP)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)
include(YarpInstallationHelpers) # pick up yarp's cmake scripts


# Include directories
include_directories(${FORCE_DIMENSION_HOME}/include
					${YARP_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES}
					  ${FORCE_DIMENSION_HOME}/lib/drdms.lib)

install(TARGETS ${PROJECTNAME} DESTINATION bin)

# ${FORCE_DIMENSION_HOME}/lib/dhdms.lib