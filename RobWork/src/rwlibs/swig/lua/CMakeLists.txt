SET(SUBSYS_NAME rw_lua )
SET(SUBSYS_DESC "Interface for accessing RobWork from lua." )
SET(SUBSYS_DEPS rw )

SET(build TRUE)

FIND_PACKAGE(SWIG 3.0.0 QUIET)
set(DEFAULT TRUE)
set(REASON )
IF( NOT SWIG_FOUND)
    set(DEFAULT false)
    set(REASON "SWIG not found!")
else()

endif()
 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
   SET(RWSIM_HAVE_LUA TRUE CACHE INTERNAL "")
	#MESSAGE(STATUS "SWIG found adding swig modules!")
	INCLUDE(UseSWIG)
	
	SET(CMAKE_SWIG_FLAGS "")

    add_custom_target(${SUBSYS_NAME}_all)

    foreach(RW_MODULE rw rw_assembly rw_control rw_pathoptimization rw_pathplanners rw_proximitystrategies rw_task)
		SET_SOURCE_FILES_PROPERTIES(../${RW_MODULE}.i PROPERTIES CPLUSPLUS ON)
		SET_SOURCE_FILES_PROPERTIES(../${RW_MODULE}.i PROPERTIES SWIG_FLAGS "-includeall")
		
	    set(SOURCE)
	    if ("${RW_MODULE}" STREQUAL "rw")
	        set(SOURCE ../ScriptTypes.cpp Lua.cpp LuaState.cpp)
	    endif()

		# lua interface generation
		IF ((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
			SWIG_ADD_LIBRARY(${RW_MODULE}_lua TYPE SHARED LANGUAGE lua SOURCES ../${RW_MODULE}.i ${SOURCE})
		ELSE()
			SWIG_ADD_MODULE(${RW_MODULE}_lua lua ../${RW_MODULE}.i ${SOURCE})
		ENDIF()
        add_dependencies(${SUBSYS_NAME}_all ${RW_MODULE}_lua)

	    IF((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
	      SET_TARGET_PROPERTIES(${RW_MODULE}_lua PROPERTIES LINK_FLAGS -Wl,--no-undefined)
	    ENDIF()
	
	    IF ((CMAKE_VERSION VERSION_GREATER 3.12.0) OR (CMAKE_VERSION VERSION_EQUAL 3.12.0))
	        SWIG_ADD_LIBRARY(${RW_MODULE}_lua_s TYPE STATIC LANGUAGE lua SOURCES ../${RW_MODULE}.i ${SOURCE})
	    ELSE()
	        ADD_LIBRARY(${RW_MODULE}_lua_s STATIC Lua.cpp LuaState.cpp ${swig_generated_sources} ${swig_other_sources})
	        ADD_DEPENDENCIES(${RW_MODULE}_lua_s rw_lua) # avoid using the source files before they have been generated
	    ENDIF()
        add_dependencies(${SUBSYS_NAME}_all ${RW_MODULE}_lua_s)

		# this is used to indicate static linking to Visual Studio or mingw
		IF (DEFINED MSVC) 
		    SET_TARGET_PROPERTIES(${RW_MODULE}_lua_s PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
		ELSE()
		    SET_TARGET_PROPERTIES(${RW_MODULE}_lua_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
		ENDIF()
	
	    install(TARGETS ${RW_MODULE}_lua ${RW_MODULE}_lua_s 
	        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT swig
	        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT swig
	        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT swig)
    endforeach()

    SWIG_LINK_LIBRARIES(rw_lua rw ${LUA_LIBRARIES})
    SWIG_LINK_LIBRARIES(rw_assembly_lua rw_assembly rw ${LUA_LIBRARIES})
    SWIG_LINK_LIBRARIES(rw_control_lua rw_control rw ${LUA_LIBRARIES})
    SWIG_LINK_LIBRARIES(rw_pathoptimization_lua rw_pathoptimization rw ${LUA_LIBRARIES})
    SWIG_LINK_LIBRARIES(rw_pathplanners_lua rw_pathplanners rw ${LUA_LIBRARIES})
    SWIG_LINK_LIBRARIES(rw_proximitystrategies_lua rw_proximitystrategies rw ${LUA_LIBRARIES})
    SWIG_LINK_LIBRARIES(rw_task_lua rw_task rw ${LUA_LIBRARIES})
    TARGET_LINK_LIBRARIES(rw_lua_s rw)
    TARGET_LINK_LIBRARIES(rw_assembly_lua_s rw_assembly rw_lua_s)
    TARGET_LINK_LIBRARIES(rw_control_lua_s rw_control rw_lua_s)
    TARGET_LINK_LIBRARIES(rw_pathoptimization_lua_s rw_pathoptimization rw_lua_s)
    TARGET_LINK_LIBRARIES(rw_pathplanners_lua_s rw_pathplanners rw_lua_s)
    TARGET_LINK_LIBRARIES(rw_proximitystrategies_lua_s rw_proximitystrategies rw_lua_s)
    TARGET_LINK_LIBRARIES(rw_task_lua_s rw_task rw_lua_s)
ENDIF()
