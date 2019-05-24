#MESSAGE("ROBWORKSTUDIO LIBS: ${ROBWORKSTUDIO_LIBRARIES}")

# First we compile the robworkstudio library
ADD_SUBDIRECTORY(rws)

# compile the component libraries (plugins and other static libs) 
ADD_SUBDIRECTORY(rwslibs)

# Then sandbox if choosen
OPTION(RWS_BUILD_SANDBOX "Set when you want to build the sandbox library" ${RWS_BUILD_SANDBOX})
IF ( RWS_BUILD_SANDBOX )
    ADD_SUBDIRECTORY(sandbox)
ENDIF()

# Propagate libraries to parent directory
set(RWS_PLUGIN_LIBRARIES ${RWS_PLUGIN_LIBRARIES} PARENT_SCOPE)
set(RWS_COMPONENT_LIBRARIES ${RWS_COMPONENT_LIBRARIES} PARENT_SCOPE)

# now create the actual RobWorkStudio executable
# Now the RobWorkStudio executable need to be build:
IF (WIN32 AND IS_RELEASE)
	ADD_EXECUTABLE(RobWorkStudio WIN32 main.cpp )
ELSE(WIN32 AND IS_RELEASE)
	ADD_EXECUTABLE(RobWorkStudio main.cpp )
ENDIF(WIN32 AND IS_RELEASE)
SET(LIBS_TMP ${RWS_PLUGIN_LIBRARIES} rws ${RWS_COMPONENT_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})
#MESSAGE("${LIBS_TMP} rws_rwstudioapp ${ROBWORK_LIBRARIES}")
TARGET_LINK_LIBRARIES(RobWorkStudio  ${LIBS_TMP} rws_rwstudioapp ${ROBWORK_LIBRARIES})

IF (RWS_USE_STATIC_LINK_PLUGINS)
    ADD_DEPENDENCIES(RobWorkStudio rws ${RWS_COMPONENT_LIBRARIES} )
ENDIF()

# Some install stuff
INSTALL(TARGETS RobWorkStudio DESTINATION ${BIN_INSTALL_DIR})
IF(RWS_USE_STATIC_LINK_PLUGINS)
    INSTALL(FILES "${RWS_ROOT}/bin/RobWorkStudio.ini.template.static" 
	        DESTINATION ${BIN_INSTALL_DIR}
            RENAME "RobWorkStudio.ini.template" 
    )
    CONFIGURE_FILE("${RWS_ROOT}/bin/RobWorkStudio.ini.template.static"
                   "${RWS_ROOT}/bin/${RWS_BUILD_TYPE}/RobWorkStudio.ini.template"
                   COPYONLY
    )    
ELSE()
    INSTALL(FILES "${RWS_ROOT}/bin/RobWorkStudio.ini.shared"
	        DESTINATION ${BIN_INSTALL_DIR}
	        RENAME "RobWorkStudio.ini"
    )
    #CONFIGURE_FILE("${RWS_ROOT}/bin/RobWorkStudio.ini.shared"
    #               "${RWS_ROOT}/bin/${RWS_BUILD_TYPE}/RobWorkStudio.ini"
    #               COPYONLY
    #)
    FILE(GENERATE OUTPUT $<TARGET_FILE_DIR:RobWorkStudio>/RobWorkStudio.ini
    	INPUT "${RWS_ROOT}/bin/RobWorkStudio.ini.shared"
    )
ENDIF()

# Install all headerfiles
INSTALL(DIRECTORY rws DESTINATION ${INCLUDE_INSTALL_DIR} 
    FILES_MATCHING 
        PATTERN "*.h" 
        PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE
)
