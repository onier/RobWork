cmake_minimum_required(VERSION 2.6.0)
PROJECT(UR_RTDE_TEST)

#Include default settings for constructing a robwork dependent project
set(RobWorkHardware_DIR "$ENV{RWHW_ROOT}/cmake")

find_package(RobWorkHardware COMPONENTS universalrobots_rtde REQUIRED HINTS "../../cmake")
find_package(Threads REQUIRED)

include_directories(${ROBWORKHARDWARE_INCLUDE_DIRS})
link_directories( ${ROBWORKHARDWARE_LIBRARY_DIRS})

add_executable(ur_rtde_test ur_rtde_test.cpp)
target_link_libraries(ur_rtde_test ${ROBWORKHARDWARE_LIBRARIES} ${ROBWORK_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

