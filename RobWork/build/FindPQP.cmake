# - Find Xerces-C
# Find the Xerces-C includes and library
# if you want to specify the location of xerces then set PQP_ROOT or PQP_INCLUDE_DIR and
# XERCES_LIB_DIR. If they are not set then they will try to be resolved automaticaly
#  
#  PQP_ROOT        - Hint as to where to find include and lib dirs (Not supported yet)
#  PQP_INCLUDE_DIR - Where to find PQP include sub-directory.
#  PQP_LIBRARY_DIR     - Where to find PQP lib sub-directory.
#  PQP_LIBRARIES   - List of libraries when using Xerces-C.
#  PQP_FOUND       - True if Xerces-C found.

IF (PQP_INCLUDE_DIR)
  # Already in cache, be silent.
  SET(PQP_FIND_QUIETLY TRUE)
ENDIF (PQP_INCLUDE_DIR)

FIND_PATH(PQP_INCLUDE_DIR NAMES "PQP.h" PATHS ${PQP_INCLUDE_DIR})

SET(PQP_NAMES pqp)
FIND_LIBRARY(PQP_LIBRARY NAMES ${PQP_NAMES} PATHS ${PQP_LIBRARY_DIR})



# Handle the QUIETLY and REQUIRED arguments and set PQP_FOUND to
# TRUE if all listed variables are TRUE.
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
  PQP DEFAULT_MSG
  PQP_LIBRARY PQP_INCLUDE_DIR
)

IF(PQP_FOUND)
  SET( PQP_LIBRARIES ${PQP_LIBRARY} )
ELSE(PQP_FOUND)
  SET( PQP_LIBRARIES )
ENDIF(PQP_FOUND)

MARK_AS_ADVANCED( PQP_LIBRARY PQP_INCLUDE_DIR )
