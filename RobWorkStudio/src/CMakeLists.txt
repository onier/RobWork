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
    ADD_SUBDIRECTORY(sandbox)
ENDIF ()
	
SET(RWS_PLUGIN_LIBRARIES $ENV{RWS_PLUGIN_LIBRARIES} CACHE string "plugin libs")
SET(RWS_COMPONENT_LIBRARIES $ENV{RWS_COMPONENT_LIBRARIES} CACHE string "component libs")

# now create the actual RobWorkStudio executable

# Now the RobWorkStudio executable need to be build:
ADD_EXECUTABLE(RobWorkStudio main.cpp ProgramOptions.cpp)
SET(LIBS_TMP ${RWS_PLUGIN_LIBRARIES} ${RWS_COMPONENT_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})
MESSAGE("LIBS_TMP = ${LIBS_TMP}")
TARGET_LINK_LIBRARIES(RobWorkStudio ${LIBS_TMP})
#MESSAGE(" ${LIBS_TMP} ")
#MESSAGE(" ${RWS_PLUGIN_LIBRARIES} ")

# Some install stuff
INSTALL(TARGETS RobWorkStudio DESTINATION bin)
INSTALL(FILES "${RWSTUDIO_ROOT}/bin/RobWorkStudio.ini_template" 
        DESTINATION bin
        RENAME "RobWorkStudio.ini"
)

# this will configure the dependency file that will be used if install is invoked
GET_TARGET_PROPERTY(rwstudio_executable_name RobWorkStudio LOCATION)
CONFIGURE_FILE(
   "${RWSTUDIO_ROOT}/build/dependencies.cmake.in"
   "${RWSTUDIO_ROOT}/build/dependencies.cmake"
   @ONLY
)
#INSTALL(SCRIPT "${RWSTUDIO_ROOT}/build/dependencies.cmake")

#include(${RWSTUDIO_ROOT}/build/dependencies.cmake)

