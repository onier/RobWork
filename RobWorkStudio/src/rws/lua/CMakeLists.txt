
IF( RWS_HAVE_LUA )
    SET(SRC_FILES )
    SET(SRC_FILES_HPP )
    ADD_TOLUA_PACKAGE("LuaRWStudio" "LuaRWStudioStub")
    ADD_LIBRARY(rwstudio_lua ${SRC_FILES} ${SRC_FILES_HPP})

    INSTALL(TARGETS rwstudio_lua DESTINATION lib)
    INSTALL(FILES ${SRC_FILES} DESTINATION include/rwlibs/lua)
ENDIF()
