SET(SUBSYS_NAME rwsim_python )
SET(SUBSYS_DESC "Interface for accessing RobWorkSim from python." )
SET(SUBSYS_DEPS rw )

SET(build TRUE)

FIND_PACKAGE(SWIG 3.0.0 QUIET)

set(DEFAULT TRUE)
set(REASON )
IF( NOT SWIG_FOUND)
    set(DEFAULT false)
    set(REASON "SWIG not found!")
else()

    FIND_PACKAGE(PythonInterp QUIET)
    FIND_PACKAGE(PythonLibs QUIET)

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

    SET_SOURCE_FILES_PROPERTIES(../rwsim.i PROPERTIES CPLUSPLUS ON)
	SET_SOURCE_FILES_PROPERTIES(../rwsim.i PROPERTIES SWIG_FLAGS "-includeall")
    if (NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
    	SET_SOURCE_FILES_PROPERTIES(../rw.i PROPERTIES SWIG_FLAGS "-includeall;-doxygen")
    endif()
	
    SET(CMAKE_SWIG_OUTDIR ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY})
	IF ((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
		SWIG_ADD_LIBRARY(${LIB_PREFIX}rwsim TYPE MODULE LANGUAGE python SOURCES ../rwsim.i ../ScriptTypes.cpp)
	ELSE()
		SWIG_ADD_MODULE(rwsim python ../rwsim.i ../ScriptTypes.cpp)
	ENDIF()
    if(POLICY CMP0078)
        set(TARGET_NAME ${LIB_PREFIX}rwsim)
    else()
        set(TARGET_NAME ${SWIG_MODULE_rwsim_REAL_NAME})
    endif()
    SWIG_LINK_LIBRARIES(${LIB_PREFIX}rwsim
        ${RWSIM_ODE_LIBRARY}
        rwsim
        ${ROBWORK_LIBRARIES}
        ${PYTHON_LIBRARIES} )
    IF((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
      SET_TARGET_PROPERTIES(${TARGET_NAME} PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    ENDIF()

    if (NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
        find_program(pydoc pydoc)
        if (pydoc)
            add_custom_command(TARGET ${TARGET_NAME}
                POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/../../../RobWork/libs/${RW_BUILD_TYPE} ${pydoc} -w rwsim
                WORKING_DIRECTORY ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}
                COMMENT "Creating pydoc..."
            )
        endif()
    endif()
ENDIF()
