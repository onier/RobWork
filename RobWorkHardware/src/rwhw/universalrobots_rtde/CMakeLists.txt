SET(SUBSYS_NAME universalrobots_rtde)
SET(SUBSYS_DESC "Driver for universal robot using the newer RTDE interface." )
SET(SUBSYS_DEPS  )

set(DEFAULT TRUE)
set(REASON)

set(RTDE_VERSION_REQUIRED 1.0.0)

find_package(ur_rtde ${RTDE_VERSION_REQUIRED} QUIET CONFIG)
if(NOT ur_rtde_FOUND)
	# Try the internal one
	option(PYTHON_BINDINGS "Build python bindings for libraries" OFF)
	add_subdirectory(${RWHW_ROOT}/ext/ur_rtde ur_rtde_ext)
	set(DEFAULT TRUE)
	set(REASON "Building internal ur_rtde library")
else()
	set(DEFAULT TRUE)
	set(REASON "Found ur_rtde library outside RobWorkHardware")
endif()

RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
	#find_package(ur_rtde ${RTDE_VERSION_REQUIRED} REQUIRED CONFIG)

    SET(SRC_CPP URRTDE.cpp)
    SET(SRC_HPP URRTDE.hpp)
    
    RW_ADD_LIBRARY(rwhw_universalrobots_rtde universalrobots_rtde ${SRC_CPP} ${SRC_HPP})
    TARGET_LINK_LIBRARIES(rwhw_universalrobots_rtde PRIVATE ur_rtde::rtde ${ROBWORK_LIBRARIES})
    RW_ADD_INCLUDES(universalrobots_rtde "rwhw/universalrobots_rtde" ${SRC_HPP})

    SET( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_universalrobots_rtde;")
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ENDIF()
