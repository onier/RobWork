add_subdirectory(lua)
add_subdirectory(java)
add_subdirectory(python)

# Propagate libraries to parent directory
set(RWS_COMPONENT_LIBRARIES ${RWS_COMPONENT_LIBRARIES} PARENT_SCOPE)