#MESSAGE("ROBWORKSTUDIO LIBS: ${ROBWORKSTUDIO_LIBRARIES}")


# First we compile the robworkstudio library
ADD_SUBDIRECTORY(rws)

# then the components 
SET(ENV{RWS_COMPONENT_LIBRARIES} "")
ADD_SUBDIRECTORY(components)

# then the plugins 
SET(ENV{RWS_PLUGIN_LIBRARIES} "")
ADD_SUBDIRECTORY(plugins)

# Then sandbox if choosen
OPTION(RWS_BUILD_SANDBOX "Set when you want to build the sandbox library" ${RWS_BUILD_SANDBOX})
IF ( RWS_BUILD_SANDBOX )
    MESSAGE(STATUS "ADDING SANDBOX")
    ADD_SUBDIRECTORY(sandbox)
ENDIF ()

SET(RWS_PLUGIN_LIBRARIES $ENV{RWS_PLUGIN_LIBRARIES} CACHE string "plugin libs")
SET(RWS_COMPONENT_LIBRARIES $ENV{RWS_COMPONENT_LIBRARIES} CACHE string "component libs")

# now create the actual RobWorkStudio executable

# Now the RobWorkStudio executable need to be build:
ADD_EXECUTABLE(RobWorkStudio main.cpp ProgramOptions.cpp)
SET(LIBS_TMP ${RWS_PLUGIN_LIBRARIES} ${RWS_COMPONENT_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})
TARGET_LINK_LIBRARIES(RobWorkStudio ${LIBS_TMP})
MESSAGE(" ${LIBS_TMP} ")
MESSAGE(" ${RWS_PLUGIN_LIBRARIES} ")
# 
# Some install stuff
INSTALL(TARGETS RobWorkStudio DESTINATION bin)
INSTALL(FILES "${RWSTUDIO_ROOT}/bin/RobWorkStudio.ini_template" 
        DESTINATION bin
        RENAME "RobWorkStudio.ini"
)

# -------------------------------------------------------------------------
# this is used to set up installation of dlls/so's that RobWorkStudio 
# depends on
SET(DEPENDS_DIRS 
    "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}" 
    "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}" 
#    ${QT_LIBRARIES} 
     ${QT_BINARY_DIR}
     ${QT_PLUGINS_DIR}
)

SET(target "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/RobWorkStudio.exe")
SET(EXEPATH "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

include(InstallRequiredSystemLibraries)
INCLUDE(GetPrerequisites OPTIONAL RESULT_VARIABLE prereq_result)

#set(TARGET_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Debug)
set(PREREQ "")
get_prerequisites("${target}" PREREQ 0 1 ${EXEPATH} "")
message(STATUS "PREREQ=${PREREQ}")


IF( prereq_result ) 
    IF(WIN32)
        SET(RWS_DEPENDENCIES 
            "QtCore4.dll;QtGui4.dll;QtOpenGL4.dll;MINGWM10.dll"
            "MSVCRT.dll;GLU32.dll"
        )
        IF(USE_XERCES)
        	#LIST(APPEND RW_DEPENDENCIES libxerces-c2_8_0.dll)
        	MESSAGE("Xerces libraries: ${XERCESC_LIBRARY}")
        ENDIF()
        
        
        
        FOREACH(item ${RWS_DEPENDENCIES})
            gp_resolve_item("${target}" "${item}" "${EXEPATH}" "${DEPENDS_DIRS}" resolved_item)
            INSTALL(FILES "${resolved_item}" DESTINATION bin)
            #MESSAGE("${resolved_item}")
        ENDFOREACH()
    ENDIF ()
ENDIF()
