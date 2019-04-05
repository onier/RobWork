# Subdirectories to process.
add_subdirectory(rwhw)

if (DEFINED COMPILE_SANDBOX)
  add_subdirectory(sandbox)
endif ()

# Propagate libraries to parent directory
set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} PARENT_SCOPE)