###############################################################################
# Add a library target.
# _name The library name.
# _component The part of RW that this library belongs to.
# ARGN The source files for the library.
MACRO(RWS_ADD_PLUGIN _name _component _lib_type)
    ADD_LIBRARY(${_name} ${_lib_type} ${ARGN})
    # must link explicitly against boost.
    target_link_libraries(${_name} ${Boost_LIBRARIES})
    
    # Only link if needed
    if(WIN32 AND MSVC)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
    elseif(__COMPILER_PATHSCALE)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -mp)
    else()
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl,--as-needed)
    endif()
    
    
    # The library to build:
    #IF (RWS_USE_STATIC_LINK_PLUGINS)
    #  ADD_LIBRARY(${TargetName} STATIC ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
    #  INSTALL(TARGETS ${TargetName} DESTINATION ${LIB_INSTALL_DIR})
    #ELSE ()
    #  SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    #  ADD_LIBRARY(${TargetName} SHARED ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
    #  SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${RWS_LIBRARY_OUT_DIR})
    #  # Link the standard static libraries with with the shared library:
    #  TARGET_LINK_LIBRARIES(${TargetName} ${ROBWORKSTUDIO_LIBRARIES})
    #  INSTALL(TARGETS ${TargetName} DESTINATION ${BIN_INSTALL_DIR})
    #ENDIF ()
    IF( "${_lib_type}" STREQUAL "STATIC" )
    SET(ENV{RWS_PLUGIN_LIBRARIES} "$ENV{RWS_PLUGIN_LIBRARIES}${_name};")
    ENDIF()
    set_target_properties(${_name} PROPERTIES
        VERSION ${ROBWORKSTUDIO_VERSION}
        SOVERSION ${ROBWORKSTUDIO_VERSION_MAJOR}.${ROBWORKSTUDIO_VERSION_MINOR}
        #DEFINE_SYMBOL "RWAPI_EXPORTS"
        )
    #if(USE_PROJECT_FOLDERS)
    #  set_target_properties(${_name} PROPERTIES FOLDER "Libraries")
    #endif(USE_PROJECT_FOLDERS)

    install(TARGETS ${_name}
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT ${_component}
        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_component}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_component})

endmacro()

###############################################################################
# Add a library target.
# _name The library name.
# _component The part of RW that this library belongs to.
# ARGN The source files for the library.
MACRO(RWS_ADD_COMPONENT _name _component)
    ADD_LIBRARY(${_name} ${PROJECT_LIB_TYPE} ${ARGN})
    # must link explicitly against boost.
    target_link_libraries(${_name} ${Boost_LIBRARIES})
    
    SET(ENV{RWS_COMPONENT_LIBRARIES} "$ENV{RWS_COMPONENT_LIBRARIES}${_name};")
    
    # Only link if needed
    if(WIN32 AND MSVC)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
    elseif(__COMPILER_PATHSCALE)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -mp)
    else()
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl,--as-needed)
    endif()
    #
    set_target_properties(${_name} PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        #DEFINE_SYMBOL "RWAPI_EXPORTS"
        )
    #if(USE_PROJECT_FOLDERS)
    #  set_target_properties(${_name} PROPERTIES FOLDER "Libraries")
    #endif(USE_PROJECT_FOLDERS)

    install(TARGETS ${_name}
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT ${_component}
        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_component}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_component})

endmacro()

#
# Setting up macro for easily adding rws plugin target 
#
MACRO (RWS_QT4_WRAP_UI outfiles )
QT4_EXTRACT_OPTIONS(ui_files ui_options ${ARGN})

FOREACH (it ${ui_files})
  GET_FILENAME_COMPONENT(outfile ${it} NAME_WE)
  GET_FILENAME_COMPONENT(infile ${it} ABSOLUTE)
  GET_FILENAME_COMPONENT(outpath ${it} PATH)
  
  SET(outfile ${CMAKE_CURRENT_SOURCE_DIR}/${outpath}/ui_${outfile}.h)
  ADD_CUSTOM_COMMAND(OUTPUT ${outfile}
    COMMAND ${QT_UIC_EXECUTABLE}
    ARGS ${ui_options} -o ${outfile} ${infile}
    MAIN_DEPENDENCY ${infile})
  SET(${outfiles} ${${outfiles}} ${outfile})
ENDFOREACH (it)

ENDMACRO (RWS_QT4_WRAP_UI)