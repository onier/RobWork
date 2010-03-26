FIND_PACKAGE(Tolua++)

SET(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})

MACRO (ADD_TOLUA_PACKAGE1 pkgname stubname)
ADD_CUSTOM_COMMAND(
  # Because we are building in a temporary directory, we have to use absolute
  # paths here.
  OUTPUT
  "${SRC_DIR}/${pkgname}Stub.cpp"
  "${SRC_DIR}/${pkgname}Stub.hpp"

  COMMAND ${TOLUA_CMD}
  -o "${SRC_DIR}/${pkgname}Stub.cpp"
  -H "${SRC_DIR}/${pkgname}Stub.hpp"
  -n "${pkgname}" "${SRC_DIR}/${pkgname}.pkg"
  DEPENDS "${SRC_DIR}/${pkgname}.pkg")

LIST(APPEND SRC_FILES
  "${pkgname}.cpp"
  "${SRC_DIR}/${pkgname}Stub.cpp"
)

LIST(APPEND SRC_FILES_HPP  
  "${pkgname}.cpp"
  "${SRC_DIR}/${pkgname}Stub.cpp"
)
ENDMACRO (ADD_TOLUA_PACKAGE1)

# if we don't find lua and use lua is enabled then we assume that it will 
# be compiled with RobWork
IF( USE_LUA )
    SET(SRC_FILES LuaRobWork.cpp)
    SET(SRC_FILES_HPP LuaRobWork.hpp)
    
    ADD_TOLUA_PACKAGE1("LuaMath" "LuaMathStub")
    ADD_TOLUA_PACKAGE1("LuaKinematics" "LuaKinematicsStub")
    ADD_TOLUA_PACKAGE1("LuaLoaders" "LuaLoadersStub")
    ADD_TOLUA_PACKAGE1("LuaSensor" "LuaLoadersStub")
    ADD_TOLUA_PACKAGE1("LuaModels" "LuaModelsStub")
    ADD_TOLUA_PACKAGE1("LuaTrajectory" "LuaTrajectoryStub") 
    ADD_TOLUA_PACKAGE1("LuaProximity" "LuaProximityStub")
    ADD_TOLUA_PACKAGE1("LuaCommon" "LuaCommonStub")
    
    ADD_LIBRARY(rw_lua ${SRC_FILES} ${SRC_FILES_HPP})
    
    INSTALL(TARGETS rw_lua DESTINATION ${RW_LIB_INSTALL_DIR})
    #INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/lua)
ENDIF()
