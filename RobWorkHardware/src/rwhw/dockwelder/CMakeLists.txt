SET(SUBSYS_NAME dockwelder )
SET(SUBSYS_DESC "Library for controlling the Dockwelder robot" )
SET(SUBSYS_DEPS rw )

SET(build TRUE)
set(DEFAULT TRUE)
set(REASON) 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
    #MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    
    INCLUDE_DIRECTORIES(${Boost_ASIO_INCLUDE_DIR}) 
    RW_ADD_LIBRARY(rwhw_dockwelder dockwelder DockWelder.cpp DockWelder.hpp)
    TARGET_LINK_LIBRARIES(rwhw_dockwelder PUBLIC ${ROBWORK_LIBRARIES})
    RW_ADD_INCLUDES(dockwelder "rwhw/dockwelder" DockWelder.hpp)
    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} rwhw_dockwelder PARENT_SCOPE)
ENDIF()
