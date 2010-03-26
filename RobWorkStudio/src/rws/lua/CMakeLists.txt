
FIND_PACKAGE(Tolua++)

ADD_TOLUA_PACKAGE("LuaRWStudio" "LuaRWStudioStub")

ADD_LIBRARY(rwstudio_lua ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rwstudio_lua DESTINATION lib)
#INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/lua)
