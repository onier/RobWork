set(Ext ${PROJECT_SOURCE_DIR}/ext)

# The camera file and include path to use.

SET(SRC_FILES
    Controller.cpp
    JointController.cpp
)

SET(SRC_FILES_HPP
    Controller.hpp
    JointController.hpp
)

ADD_LIBRARY(rw_control ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_control DESTINATION ${RW_LIB_INSTALL_DIR})
#INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/simulation)
