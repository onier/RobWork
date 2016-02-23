# - Find Assimp
# Find the Assimp includes and library
# if you want to specify the location of assimp then set ASSIMP_ROOT or ASSIMP_INCLUDE_DIR and
# ASSIMP_LIB_DIR. If they are not set then they will try to be resolved automaticaly
#  
#  ASSIMP_ROOT        - Hint as to where to find include and lib dirs (Not supported yet)
#  ASSIMP_INCLUDE_DIR - Where to find Assimp include sub-directory.
#  ASSIMP_LIBRARY_DIR     - Where to find Assimp lib sub-directory.
#  ASSIMP_LIBRARIES   - List of libraries when using Assimp.
#  ASSIMP_FOUND       - True if Assimp found.

IF (ASSIMP_INCLUDE_DIR)
    # Already in cache, be silent.
    SET(ASSIMP_FIND_QUIETLY TRUE)
ENDIF(ASSIMP_INCLUDE_DIR)

SET(ASSIMP_INCLUDE_DIR "/usr/include/assimp")
SET(ASSIMP_LIBRARY_DIR "/usr/lib")

FIND_PATH(ASSIMP_INCLUDE_DIR_TMP "Importer.hpp" PATHS ${ASSIMP_INCLUDE_DIR})

SET(ASSIMP_NAMES assimp)
FIND_LIBRARY(ASSIMP_LIBRARY NAMES ${ASSIMP_NAMES} PATHS ${ASSIMP_LIBRARY_DIR} )
# Handle the QUIETLY and REQUIRED arguments and set ASSIMP_FOUND to
# TRUE if all listed variables are TRUE.
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
  ASSIMP DEFAULT_MSG
  ASSIMP_LIBRARY 
  ASSIMP_INCLUDE_DIR_TMP
)

IF(ASSIMP_FOUND)
  SET( ASSIMP_LIBRARIES ${ASSIMP_LIBRARY} )
  SET( ASSIMP_INCLUDE_DIRS ${ASSIMP_INCLUDE_DIR_TMP} )
ELSE(ASSIMP_FOUND)
  SET( ASSIMP_LIBRARIES )
  SET( ASSIMP_INCLUDE_DIRS )
ENDIF(ASSIMP_FOUND)

MARK_AS_ADVANCED( ASSIMP_LIBRARY ASSIMP_INCLUDE_DIR )
