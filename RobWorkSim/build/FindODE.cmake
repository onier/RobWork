# Locate ode
# This module defines
# ODE_LIBRARY
# ODE_FOUND, if false, do not try to link to ode 
# ODE_INCLUDE_DIR, where to find the headers
#
# this module optionally use followin vars to guide the search for ODE:
# ODE_DIR - path to ODE root dir
# ODE_LIBRARY_DIR - specify to guide the search in library
# ODE_LIBRARY_NAME - specify to guide library search ad selection
# ODE_USE_SINGLE - set if force using double
# ODE_USE_DOUBLE - set if force using double
# ODE_USE_DEBUG - set if force using debug
# created by RobWork, based on code by David Guthrie.  Based on code by Robert Osfield 

FIND_PATH(ODE_INCLUDE_DIR ode/ode.h
    ${ODE_DIR}/include
    $ENV{ODE_DIR}/include
    $ENV{ODE_DIR}
    ${DELTA3D_EXT_DIR}/inc
    $ENV{DELTA_ROOT}/ext/inc
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/include
    /usr/include
    /usr/include/cal3d
    /sw/include # Fink
    /opt/local/include # DarwinPorts
    /opt/csw/include # Blastwave
    /opt/include
    [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSG_ROOT]/include
    /usr/freeware/include
)


MACRO(FIND_ODE_LIBRARY MYLIBRARY MYLIBRARYNAME)
    FIND_LIBRARY(${MYLIBRARY}
        NAMES ${ODE_LIBRARY_NAME} ${MYLIBRARYNAME}
        PATHS
        ${ODE_LIBRARY_DIR}
        ${ODE_DIR}/lib
        $ENV{ODE_DIR}/lib
        $ENV{ODE_DIR}
        ${DELTA3D_EXT_DIR}/lib
        $ENV{DELTA_ROOT}/ext/lib
        ~/Library/Frameworks
        /Library/Frameworks
        /usr/local/lib
        /usr/lib
        /sw/lib
        /opt/local/lib
        /opt/csw/lib
        /opt/lib
        [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSG_ROOT]/lib
        /usr/freeware/lib64
        PATH_SUFFIXES
        lib/ReleaseSingleLib
        lib/ReleaseSingleDLL
        lib/ReleaseDoubleLib
        lib/ReleaseDoubleDLL
        lib/DebugSingleLib
        lib/DebugSingleDLL
        lib/DebugDoubleLib
        lib/DebugDoubleDLL
        ReleaseSingleLib
        ReleaseSingleDLL
        ReleaseDoubleLib
        ReleaseDoubleDLL        
        DebugSingleLib
        DebugSingleDLL
        DebugDoubleLib
        DebugDoubleDLL
    )
    
ENDMACRO(FIND_ODE_LIBRARY MYLIBRARY MYLIBRARYNAME)

IF(ODE_USE_DEBUG)
    IF(ODE_USE_SINGLE)
        SET(DEBUG_LIST ode_singled oded )    
    ELSEIF(ODE_USE_DOUBLE)
        SET(DEBUG_LIST ode_doubled oded )
    ELSE()
        SET(DEBUG_LIST oded ode_singled ode_doubled)
    ENDIF()    
    FIND_ODE_LIBRARY(ODE_LIBRARY "${DEBUG_LIST}")
    
ELSE()
    
    IF(ODE_USE_SINGLE)
        SET(RELEASE_LIST ode_single ode )
        SET(DEBUG_LIST ode_singled oded )
    ELSEIF(ODE_USE_DOUBLE)
        SET(RELEASE_LIST ode_double ode )
        SET(DEBUG_LIST ode_doubled oded )
    ELSE()
        SET(RELEASE_LIST ode ode_single ode_double)
        SET(DEBUG_LIST oded ode_singled ode_doubled)
    ENDIF()        
    
    FIND_ODE_LIBRARY(ODE_LIBRARY "${RELEASE_LIST}")
    
    IF( NOT ODE_LIBRARY )
        FIND_ODE_LIBRARY(ODE_LIBRARY "${DEBUG_LIST}")
    ENDIF()
ENDIF()

#MESSAGE("${ODE_INCLUDE_DIR}")
#MESSAGE("${ODE_LIBRARY_DEBUG}")

SET(ODE_FOUND NO)
IF(ODE_LIBRARY AND ODE_INCLUDE_DIR)
    SET(ODE_FOUND YES)
ENDIF(ODE_LIBRARY AND ODE_INCLUDE_DIR)



