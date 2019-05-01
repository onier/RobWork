# Test CMake version
cmake_minimum_required(VERSION 2.8.12)

CMAKE_POLICY(SET CMP0011 NEW)
CMAKE_POLICY(SET CMP0012 NEW)


# The name of the project.
PROJECT(RobWorkProject)

SET(RWP_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
SET(RW_ROOT ${ROOT}/RobWork/)
SET(RWS_ROOT ${ROOT}/RobWorkStudio/)
SET(RWSIM_ROOT ${ROOT}/RobWorkSim/)
SET(RWHW_ROOT ${ROOT}/RobWorkHardware/)

add_subdirectory(RobWork)
add_subdirectory(RobWorkHardware)
add_subdirectory(RobWorkStudio)
add_subdirectory(RobWorkSim)

########################################################33
# now configure documentation
CONFIGURE_FILE(${ROOT}/doc/Doxyfile.in ${ROOT}/doc/Doxyfile)

FIND_PROGRAM(doc_executable doxygen
    HINTS
        ${DOXYGEN_PATH}
        $ENV{DOXYGEN_PATH}
)
IF(doc_executable)
    ADD_CUSTOM_TARGET(rwdoc ${doc_executable} WORKING_DIRECTORY ${ROOT}/doc)
ENDIF()

# Packaging
#INCLUDE(build/packing.cmake)

