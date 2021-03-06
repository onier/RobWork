SET(SUBSYS_NAME rws_python )
SET(SUBSYS_DESC "Interface for accessing RobWorkStudio from python." )
SET(SUBSYS_DEPS rw )

SET(build TRUE)

FIND_PACKAGE(SWIG 3.0.0 QUIET)

set(DEFAULT TRUE)
set(REASON )
IF( NOT SWIG_FOUND)
    set(DEFAULT false)
    set(REASON "SWIG not found!")
else()

	# Prefer Python 3
	find_package(PythonInterp 3 QUIET)
	find_package(PythonLibs 3 QUIET)
	
	if (NOT PYTHONINTERP_FOUND)
	    find_package(PythonInterp QUIET)
	endif()
	if (NOT PythonLibs)
	    find_package(PythonLibs QUIET)
	endif()
	
	if(PYTHONINTERP_FOUND)
	    message(STATUS "Found Python interpreter ${PYTHON_VERSION_STRING}")
	endif()
	if(PYTHONLIBS_FOUND)
	    message(STATUS "Found Python libraries ${PYTHONLIBS_VERSION_STRING}")
	endif()

    if( NOT (PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND) )
        set(DEFAULT false)
        set(REASON "PYTHONLIBS AND PYTHONINTERP not found!")
    endif()
endif()
 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
INCLUDE(UseSWIG)
    INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
    if(POLICY CMP0078)
        set(LIB_PREFIX "_")
    else()
        set(LIB_PREFIX "")
    endif()

    SET_SOURCE_FILES_PROPERTIES(../rws.i PROPERTIES CPLUSPLUS ON)
	SET_SOURCE_FILES_PROPERTIES(../rws.i PROPERTIES SWIG_FLAGS "-includeall")
    if (NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
    	SET_SOURCE_FILES_PROPERTIES(../rw.i PROPERTIES SWIG_FLAGS "-includeall;-doxygen")
    endif()
	
	SET(CMAKE_SWIG_OUTDIR ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY})
	IF ((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
		SWIG_ADD_LIBRARY(${LIB_PREFIX}rws TYPE MODULE LANGUAGE python SOURCES ../rws.i ../ScriptTypes.cpp)
	ELSE()
		SWIG_ADD_MODULE(rws python ../rws.i ../ScriptTypes.cpp)
	ENDIF()
    if(POLICY CMP0078)
        set(TARGET_NAME ${LIB_PREFIX}rws)
    else()
        set(TARGET_NAME ${SWIG_MODULE_rws_REAL_NAME})
    endif()
    SWIG_LINK_LIBRARIES(${LIB_PREFIX}rws
        ${RWS_COMPONENT_LIBRARIES} 
        ${RWS_PLUGIN_LIBRARIES} 
        rws
        ${ROBWORK_LIBRARIES} 
        ${PYTHON_LIBRARIES} )
    IF((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
      SET_TARGET_PROPERTIES(${TARGET_NAME} PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    ENDIF()

    if (NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
        find_program(pydoc pydoc3)
        if (NOT pydoc)
            find_program(pydoc pydoc)
        endif()
        if (pydoc)
            add_custom_command(TARGET ${TARGET_NAME}
                POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/../../../RobWork/libs/${RW_BUILD_TYPE} ${pydoc} -w rws
                WORKING_DIRECTORY ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}
                COMMENT "Creating pydoc..."
            )
        endif()
    endif()
     
ENDIF()
