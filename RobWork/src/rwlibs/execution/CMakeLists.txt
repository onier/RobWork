SET(SUBSYS_NAME rw_execution)
set(SUBSYS_DESC "Execution framework")
SET(SUBSYS_DEPS rw)

SET(DEFAULT TRUE)
SET(REASON "")

SET(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)
    # The camera file and include path to use.
    SET(SRC_FILES
      ./ExecutionResult.cpp
      ./ExecutionTask.cpp
      ./Executor.cpp
      ./Platform.cpp
      ./PlatformRegistry.cpp
      ./TaskDB.cpp
    )
    
    SET(SRC_FILES_HPP
      ./ExecutionResult.hpp
      ./ExecutionTask.hpp
      ./Executor.hpp
      ./Platform.hpp
      ./PlatformRegistry.hpp
      ./TaskDB.hpp
    )
      	  
    RW_ADD_LIBRARY(${SUBSYS_NAME} execution ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(execution "rwlibs/execution" ${SRC_FILES_HPP})
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()