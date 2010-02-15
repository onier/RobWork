cmake_minimum_required(VERSION 2.6.0)

PROJECT(RobWorkSim)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

#Include default settings for constructing a robwork dependent project
INCLUDE(${ROOT}/build/ProjectSetup.cmake)

#Include default settings for constructing a robwork dependent project
INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# if we have any exe files or plugins that depend on the libraries
INCLUDE_DIRECTORIES( src )

# test if Bullet exists
IF(USE_BULLET)
	#todo: call FindPackage(Bullet)
	set(RWSIM_HAVE_BULLET TRUE)
	INCLUDE_DIRECTORIES( ${BULLET_INCLUDE_DIR} )
    LINK_DIRECTORIES( ${BULLET_LIBRARY_DIRS} )
endif()

# test if ODE exists
IF( USE_ODE )
    # todo: call FindPackage(ODE)
	SET(RWSIM_HAVE_ODE TRUE)
	INCLUDE_DIRECTORIES( ${ODE_INCLUDE_DIR} )
    LINK_DIRECTORIES( ${ODE_LIBRARY_DIRS} )
ENDIF()

# Store the test results in a RWSimConfig.hpp file.
configure_file(
  ${ROOT}/src/RWSimConfig.hpp.in
  ${ROOT}/src/RWSimConfig.hpp)

configure_file(
  ${ROOT}/plugin/RWSimGuiConfig.hpp.in
  ${ROOT}/plugin/RWSimGuiConfig.hpp)


# process sub directories
add_subdirectory(src)
add_subdirectory(plugin)
add_subdirectory(test)
