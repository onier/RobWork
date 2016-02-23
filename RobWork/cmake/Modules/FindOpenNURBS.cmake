# - Find openNURBS
# Find the openNURBS includes and library
# if you want to specify the location of openNURBS then set OPENNURBS_ROOT or OPENNURBS_INCLUDE_DIR and
# OPENNURBS_LIBRARY_DIR. If they are not set then they will try to be resolved automaticaly
#  
#  OPENNURBS_ROOT        - Hint as to where to find include and lib dirs (Not supported yet)
#  OPENNURBS_INCLUDE_DIR - Where to find openNURBS include sub-directory.
#  OPENNURBS_LIBRARY_DIR - Where to find openNURBS lib sub-directory.
#  OPENNURBS_LIBRARIES   - List of libraries when using openNURBS.
#  OPENNURBS_FOUND       - True if openNURBS found.

IF (OPENNURBS_INCLUDE_DIR)
  # Already in cache, be silent.
  SET(OPENNURBS_FIND_QUIETLY TRUE)
ENDIF (OPENNURBS_INCLUDE_DIR)

FIND_PATH(OPENNURBS_INCLUDE_DIR_TMP NAMES "opennurbs.h" PATHS ${OPENNURBS_INCLUDE_DIR})

SET(OPENNURBS_NAMES openNURBS)
FIND_LIBRARY(OPENNURBS_LIBRARY NAMES ${OPENNURBS_NAMES} PATHS ${OPENNURBS_LIBRARY_DIR})

# Handle the QUIETLY and REQUIRED arguments and set OPENNURBS_FOUND to
# TRUE if all listed variables are TRUE.
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
  OPENNURBS DEFAULT_MSG
  OPENNURBS_LIBRARY 
  OPENNURBS_INCLUDE_DIR_TMP
)

IF(OPENNURBS_FOUND)
  SET( OPENNURBS_LIBRARIES ${OPENNURBS_LIBRARY} )
  SET( OPENNURBS_INCLUDE_DIR ${OPENNURBS_INCLUDE_DIR_TMP} )
ELSE(OPENNURBS_FOUND)
  SET( OPENNURBS_LIBRARIES )
  SET( OPENNURBS_INCLUDE_DIR  )
ENDIF(OPENNURBS_FOUND)

MARK_AS_ADVANCED( OPENNURBS_LIBRARY OPENNURBS_INCLUDE_DIR )