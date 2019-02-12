# - Config file for the ur_rtde package

# Compute paths
get_filename_component(URRTDE_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET ur_rtde)
  include("${URRTDE_CMAKE_DIR}/ur_rtdeTargets.cmake")
endif()

