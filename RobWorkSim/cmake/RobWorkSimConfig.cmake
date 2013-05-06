# ------------------------------------------------------------------------------------
# Helper to use RobWorkSim from outside project
#
# ROBWORKSIM_LIBRARIES is filled with all available RobWork libraries
# ROBWORKSIM_INCLUDE_DIRS is filled with RobWorkSim and available 3rdparty headers
# ROBWORKSIM_LIBRARY_DIRS is filled with RobWorkSim components libraries  and
# 3rdparty libraries paths
# 
#                                   www.robwork.dk
#------------------------------------------------------------------------------------



### ---[ Find RobWorkSim

if(ROBWORKSIM_FIND_QUIETLY)
  set(QUIET_ QUIET)
else(ROBWORKSIM_FIND_QUIETLY)
  set(QUIET_)
endif(ROBWORKSIM_FIND_QUIETLY)

############################################## MACROS ################################################

# macro for determining the best RobWork build type match
MACRO(GET_ROBWORKSIM_BUILD_TYPE RW_ROOT RW_BUILD_TYPE)
# defaults to release 
SET(BTYPE_TMP release)
if( CMAKE_BUILD_TYPE )
  STRING(TOLOWER ${CMAKE_BUILD_TYPE} BTYPE_TMP )
endif()

# first test if the correct cmake build type is installed
if( EXISTS ${RWCFG_ROOT}/RobWorkBuildConfig_${BTYPE_TMP}.cmake )
  SET(${RW_BUILD_TYPE} ${BTYPE_TMP})
else()

  # find best robwork build match
  if(${BTYPE_TMP} STREQUAL "release")
    # find release compatible robwork installation
    if( EXISTS ${RWCFG_ROOT}/RobWorkBuildConfig_minsizerel.cmake )
      SET(${RW_BUILD_TYPE} minsizerel)
    elseif(EXISTS ${RWCFG_ROOT}/RobWorkBuildConfig_relwithdebug.cmake)
      SET(${RW_BUILD_TYPE} relwithdebug)
    elseif(EXISTS ${RWCFG_ROOT}/RobWorkBuildConfig_debug.cmake)
      SET(${RW_BUILD_TYPE} debug)
    endif()
  elseif(${BTYPE_TMP} STREQUAL "minsizerel")
    if( EXISTS ${RWCFG_ROOT}/RobWorkBuildConfig_release.cmake )
      SET(${RW_BUILD_TYPE} release)
    elseif(EXISTS ${RWCFG_ROOT}/RobWorkBuildConfig_relwithdebug.cmake)
      SET(${RW_BUILD_TYPE} relwithdebug)
    elseif(EXISTS ${RWCFG_ROOT}/RobWorkBuildConfig_debug.cmake)
      SET(${RW_BUILD_TYPE} debug)
    endif()
  elseif(${BTYPE_TMP} STREQUAL "relwithdebug")
    if(EXISTS ${RWCFG_ROOT}/RobWorkBuildConfig_release.cmake)
      SET(${RW_BUILD_TYPE} release)
    elseif( EXISTS ${RWCFG_ROOT}/RobWorkBuildConfig_minsizerel.cmake )
      SET(${RW_BUILD_TYPE} minsizerel)
    elseif(EXISTS ${RWCFG_ROOT}/RobWorkBuildConfig_debug.cmake)
      SET(${RW_BUILD_TYPE} debug)
    endif()
  elseif(${BTYPE_TMP} STREQUAL "debug")  
    if(EXISTS ${RWCFG_ROOT}/RobWorkBuildConfig_relwithdebuginfo.cmake)
      SET(${RW_BUILD_TYPE} relwithdebuginfo)
    elseif( EXISTS ${RWCFG_ROOT}/RobWorkBuildConfig_minsizerel.cmake )
      SET(${RW_BUILD_TYPE} minsizerel)
    elseif(EXISTS ${RWCFG_ROOT}/RobWorkBuildConfig_release.cmake)
      SET(${RW_BUILD_TYPE} release)
    endif()
  endif()
  MESSAGE(STATUS "warning: RobWork was not compiled with type:${BTYPE_TMP} using type:${${RW_BUILD_TYPE}} instead!")
endif()

ENDMACRO()


########################################################################################################


get_filename_component(RWSIMCFG_ROOT ${CMAKE_CURRENT_LIST_FILE} PATH)

# check if user specified a RobWorkSim_DIR
if(DEFINED RobWorkSim_DIR)
  if(EXISTS "${RobWorkSim_DIR}/src/RobWorkSimConfig.hpp")
    # Found RobWorkSim in a build tree of RobWork
    set(RWSIMCFG_ROOT "${RobWorkSim_DIR}/cmake")
    set(is_installed false)
  elseif(EXISTS "${RobWorkSim_DIR}/RobWorkConfig.cmake")
    # Found a RobWorkSim installation
    set(RWSIMCFG_ROOT "${RobWorkSim_DIR}/cmake")
    set(is_installed true)
  elseif(EXISTS "${RobWorkSim_DIR}/include/robworksim-${RobWorkSim_VERSION_MAJOR}.${RobWorkSim_VERSION_MINOR}/RobWorkSimConfig.hpp")
    set(RWSIMCFG_ROOT "${RobWorkSim_DIR}/share/robworksim-${RobWorkSim_VERSION_MAJOR}.${RobWorkSim_VERSION_MINOR}/")
    set(is_installed true)    
  else()
    # found no RobWork installation ot build tree in RobWorkSim_DIR so we try extracting it from RobWorkSimConfig.cmake location instead
  endif()
endif()

# get the relavant build type
GET_ROBWORKSIM_BUILD_TYPE(${RWSIMCFG_ROOT} RWSIM_BUILD_TYPE)


# check whether RobWorkConfig.cmake is found into a RobWork installation or in a build tree
if(EXISTS "${RWSIMCFG_ROOT}/../src/RobWorkSimConfig.hpp")
  # Found RobWorkConfig.cmake in a build tree of RobWork
  SET(succmsg "RobWorkSim: Found a RobWorkSim build tree")
  set(RWSIM_ROOT "${RWSIMCFG_ROOT}/..")
  
  set(RWSIM_INCLUDE_EXT "${RWSIM_ROOT}/ext")
  set(RWSIM_INCLUDE_SRC "${RWSIM_ROOT}/src/")
  set(RWSIM_LIBS "${RWSIM_ROOT}/libs/${RWSIM_BUILD_TYPE}/")

else()
  set(succmsg "RobWorkSim: Found a RobWorkSim installation")
  # Found a RobWork installation
  if(WIN32)
    # RobWorkConfig.cmake is installed to RWSIM_ROOT/cmake
    set(RWSIM_ROOT "${RWSIMCFG_ROOT}/..")
    set(RWSIM_INCLUDE_EXT "${RWSIM_ROOT}/ext")
    set(RWSIM_INCLUDE_SRC "${RWSIM_ROOT}/include")
    set(RWSIM_LIBS "${RWSIM_ROOT}/libs/${RWSIM_BUILD_TYPE}")
  else(WIN32)
    # RobWorkSimConfig.cmake is installed to RWSIM_INTALL/share/robworksim-x.y

    set(RWSIM_ROOT "${RWSIMCFG_ROOT}")
    set(RWSIM_INSTALL "${RWSIMCFG_ROOT}/../../")
    set(RWSIM_LIBS "${RWSIM_INSTALL}/lib/")
    SET(RWSIM_INCLUDE_SRC "${RWSIM_INSTALL}/include/robworksim-${RobWorkSim_VERSION_MAJOR}.${RobWorkSim_VERSION_MINOR}")
    set(RWSIM_INCLUDE_EXT "${RWSIM_INSTALL}/share/robworksim-${RobWorkSim_VERSION_MAJOR}.${RobWorkSim_VERSION_MINOR}/ext")
  endif(WIN32)
endif()

#MESSAGE(STATUS "RWSIM_ROOT   : ${RWSIM_ROOT}")
#MESSAGE(STATUS "RWSIMCFG_ROOT: ${RWSIMCFG_ROOT}")

#INCLUDE(${RWSIMCFG_ROOT}/RobWorkSimMacros.cmake)

#############################################################
# now RWSIM_ROOT and RWSIMCFG_ROOT is set. Lets extract the stuff needed to run a project



# next get the build configuration of the requested built type
IF(EXISTS ${RWSIM_ROOT}/cmake/RobWorkSimBuildConfig_${RWSIM_BUILD_TYPE}.cmake)
  INCLUDE(${RWSIM_ROOT}/cmake/RobWorkSimBuildConfig_${RWSIM_BUILD_TYPE}.cmake)

# setup path to custom find scripts
SET(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${RWSIMCFG_ROOT}/Modules")

# test if Bullet exists
OPTION(RWSIM_USE_BULLET "Set to ON if Bullet should be use. you may need to set BULLET_ROOT" ${RWSIM_BUILD_WITH_BULLET})
IF(RWSIM_USE_BULLET)
    IF(RWSIM_BUILD_WITH_BULLET)
        IF(NOT BULLET_ROOT) 
            SET(BULLET_ROOT ${RWSIM_BUILD_WITH_BULLET_ROOT})
        ENDIF()
        IF(NOT BULLET_INCLUDE_DIR) 
            SET(BULLET_INCLUDE_DIR ${RWSIM_BUILD_WITH_BULLET_INCLUDE_DIR})
        ENDIF()
        
        FIND_PACKAGE(Bullet)
        IF(BULLET_FOUND)
        	SET(RWSIM_BULLET_LIBRARY rwsim_bullet)
            # BULLET_LIBRARIES
            MESSAGE(STATUS "RobWorkSim: Bullet enabled and found.")
        ELSE()
            SET(RWSIM_HAVE_BULLET FALSE)
            MESSAGE(SEND_ERROR "RobWorkSim: Bullet enabled but not found. Please setup BULLET_ROOT." ${RWSIM_USE_ODE})
        ENDIF()
   ELSE()
       MESSAGE(SEND_ERROR "RobWorkSim: Bullet enabled but RobWorkSim was NOT build with Bullet support! Please recompile RobWorkSim")
   ENDIF()
ELSE()
    MESSAGE(STATUS "RobWorkSim: Bullet disabled.")
ENDIF()

# test if ODE exists
OPTION(RWSIM_USE_ODE "Set to ON if ODE should be use. you may need to set ODE_ROOT" ${RWSIM_BUILD_WITH_ODE})
IF(RWSIM_USE_ODE)
    IF(RWSIM_BUILD_WITH_ODE)
        SET(ODE_USE_DOUBLE ${RWSIM_BUILD_WITH_ODE_USE_DOUBLE})
        SET(ODE_USE_DEBUG ${RWSIM_BUILD_WITH_ODE_USE_DEBUG})
        IF(NOT ODE_DIR)
            SET(ODE_DIR ${RWSIM_BUILD_WITH_ODE_DIR})
        ENDIF()
        IF(NOT ODE_INCLUDE_DIR)
            SET(ODE_INCLUDE_DIR ${RWSIM_BUILD_WITH_ODE_INCLUDE_DIR})
        ENDIF()
        FIND_PACKAGE(ODE)
        IF(ODE_FOUND)
        	SET(RWSIM_ODE_LIBRARY rwsim_ode)
        	# ODE_LIBRARIES
            MESSAGE(STATUS "RobWorkSim: ODE enabled and found. Using ${ODE_BUILD_WITH}")
        ELSE()
            MESSAGE(SEND_ERROR "RobWorkSim: ODE enabled but not found. Please setup ODE_ROOT.")
        ENDIF()
   ELSE()
       MESSAGE(SEND_ERROR "RobWorkSim: ODE enabled but RobWorkSim was NOT build with ODE support! Please recompile RobWorkSim")
   ENDIF()
ELSE()
    MESSAGE(STATUS "RobWorkSim: ODE disabled.")
ENDIF()



set(ROBWORKSIM_BUILD_PATH "${RWSIM_BUILD_WITH_RWSIM_ROOT}")
set(ROBWORKSIM_INCLUDE_DIRS_TMP "${RWSIM_BUILD_WITH_INCLUDE_DIRS}")
set(ROBWORKSIM_LIBRARY_DIRS_TMP "${RWSIM_BUILD_WITH_LIBRARY_DIRS}")
set(ROBWORKSIM_LIBRARIES_TMP "${RWSIM_BUILD_WITH_LIBRARIES}" "${RWSIM_BUILD_WITH_PLUGIN_LIBRARIES}")



# make sure that the library and include paths are pointing to the right locations
STRING(REPLACE "${ROBWORKSIM_BUILD_PATH}/ext" "${RWSIM_INCLUDE_EXT}" ROBWORKSIM_INCLUDE_DIRS "${RWSIM_BUILD_WITH_INCLUDE_DIRS}")
STRING(REPLACE "${ROBWORKSIM_BUILD_PATH}/src" "${RWSIM_INCLUDE_SRC}" ROBWORKSIM_INCLUDE_DIRS "${ROBWORKSIM_INCLUDE_DIRS}")
list(REMOVE_DUPLICATES ROBWORKSIM_INCLUDE_DIRS)
#MESSAGE("INCLUDES: ${ROBWORKSIM_INCLUDE_DIRS}")

STRING(REPLACE "${ROBWORKSIM_BUILD_PATH}/libs/${RWSIM_BUILD_TYPE}" "${RWSIM_LIBS}" ROBWORKSIM_LIBRARY_DIRS "${RWSIM_BUILD_WITH_LIBRARY_DIRS}")
list(REMOVE_DUPLICATES ROBWORKSIM_LIBRARY_DIRS)
#MESSAGE("INCLUDES: ${ROBWORKSIM_LIBRARY_DIRS}")

STRING(REPLACE "${ROBWORKSIM_BUILD_PATH}/libs/${RWSIM_BUILD_TYPE}" "${RWSIM_LIBS}" ROBWORKSIM_LIBRARIES "${RWSIM_BUILD_WITH_LIBRARIES}")
#list(REMOVE_DUPLICATES ROBWORKSIM_LIBRARIES)
#MESSAGE("INCLUDES: ${ROBWORKSIM_LIBRARIES}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RobWorkSim DEFAULT_MSG RWSIM_ROOT ROBWORKSIM_LIBRARIES ROBWORKSIM_INCLUDE_DIRS ROBWORKSIM_LIBRARY_DIRS)
mark_as_advanced(ROBWORKSIM_LIBRARIES ROBWORKSIM_INCLUDE_DIRS ROBWORKSIM_LIBRARY_DIRS)

ELSE()
SET(ROBWORKSIM_FOUND NOTFOUND)
  MESSAGE(STATUS "This build of RobWorkSim is not compiled in ${RWSIM_BUILD_TYPE} please specify another buildtype!") 
ENDIF()


if(ROBWORKSIM_FOUND)
  set(ROBWORKSIM_VERSION ${RobWorkSim_FOUND_VERSION} CACHE STRING "RobWorkSim version")
endif(ROBWORKSIM_FOUND)