#####################################################
# Template for building RobWork dependent projects
# - the src should contain code for putting into libraries 
# - the plugins contain code for multiple RobWorkStudio plugins
# - the test for testing 
# - the example dir for illustrativ examples of your project
#
# Use config.cmake to change default settings and edit this
# file for adding additional dependencies to external projects  
#####################################################

#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project.
PROJECT(PluginUIApp)


SET(RobWorkSim_DIR ../../cmake)
FIND_PACKAGE(RobWorkSim REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORKSIM_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORKSIM_LIBRARY_DIRS} )

# now include macros from RobWorkStudio since we need them here


# if we want to use ui files add them here
SET(UIS_FILES SimTaskPlugin.ui )

RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})

QT4_WRAP_CPP(MocSrcFiles SimTaskPlugin.hpp)
# Rcc the files:

QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
SET(SrcFiles SimTaskPlugin.cpp ${UIS_OUT_H})

# The shared library to build:
ADD_LIBRARY(SimTaskPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SimTaskPlugin ${ROBWORKSIM_LIBRARIES})

