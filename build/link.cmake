# -*- cmake -*-
#message("LibraryOutput = " ${LIBRARY_OUTPUT_PATH})
set(d ${LIBRARY_OUTPUT_PATH})
link_directories(${d})
list(APPEND CMAKE_LIBRARY_PATH ${d})

# All mandatory libraries for linking with rw:
if (DEFINED MINGW)
  set(RW_UBLAS_LIBRARY_NAMES lapack blas g2c)
elseif (DEFINED MSVC)
  set(RW_UBLAS_LIBRARY_NAMES lapack_win32 blas_win32)
elseif (DEFINED UNIX)
  set(RW_UBLAS_LIBRARY_NAMES lapack)
endif ()

if (RW_HAVE_XERCES)
	find_library(XERCES_LIB xerces-c)
	message("XercesLib = "${XERCES_LIB})
	if (NOT XERCES_LIB)
	  message("Warning: Could not find Xerces library. Using default name")
	  set(XERCES_LIB xerces-c)
	endif ()
endif()

# Find pqp, and yaobi in case the user has installed these already, or
# use their raw names as defaults.
if (RW_HAVE_PQP)
	find_library(PQP_LIB pqp)
	if (NOT PQP_LIB)
	  message("Warning: Could not find PQP library. Using default name")
	  set(PQP_LIB pqp)
	endif ()
endif()

if (RW_HAVE_YAOBI)
	find_library(YAOBI_LIB yaobi)
	if (NOT YAOBI_LIB)
	  message("Warning: Could not find yaobi library. Using default name")
	  set(YAOBI_LIB yaobi)
	endif ()
endif()

# Libraries for programs using rw.
set(RW_LIBRARY_LIST
  rw
  ${RW_UBLAS_LIBRARY_NAMES}
  )

# Libraries for programs using rw_drawable.
include(FindOpenGL)
set(RW_DRAWABLE_LIBRARY_LIST
  rw_drawable
  ${RW_LIBRARY_LIST}
  ${OPENGL_LIBRARIES}
  )

# Libraries for programs using rw_lua.
include(FindOpenGL)
set(RW_LUA_LIBRARY_LIST
  rw_lua
  ${RW_LIBRARY_LIST}
  tolualib
  lualib
  )

# Libraries for programs using rw_pathplanners.
include(FindOpenGL)
set(RW_PATHPLANNERS_LIBRARY_LIST
  rw_pathplanners
  ${RW_LIBRARY_LIST}
  )

message( "-- Looking for collision libs: ")

set(CollisionDetectionLibraries)
if (RW_HAVE_PQP)
  list(APPEND CollisionDetectionLibraries ${PQP_LIB})
  message("--- PQP Found ")
else()
  message("--- PQP not found")
endif ()

if (RW_HAVE_YAOBI)
  list(APPEND CollisionDetectionLibraries ${YAOBI_LIB})
	message("--- Yaobi found")
else()
	message("--- Yaobi not found")
endif ()

# Libraries for programs using rw_proximitystrategies.
set(RW_PROXIMITYSTRATEGIES_LIBRARY_LIST
  rw_proximitystrategies
  ${RW_LIBRARY_LIST}
  ${CollisionDetectionLibraries}
  )

# etc...

if (DEFINED COMPILE_SANDBOX)
    SET(SANDBOX_LIB rw_sandbox)    
endif ()

# We should use a more standard technique for the packaging of libraries and
# their dependencies (probably there are conventions for this in CMake already).
