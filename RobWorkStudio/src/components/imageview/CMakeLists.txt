#----------------------------------------------------------------------
# Define the following variables:

# The shared library to build:
set(TargetName rwstudio_imageview)

# Standard cpp files to compile:
set(SrcFiles ImageView.cpp)

# Moc the files:
QT4_WRAP_CPP(MocSrcFiles ImageView.hpp)

# Rcc the files:
QT4_ADD_RESOURCES(RccSrcFiles "")

#----------------------------------------------------------------------
# Everything below should be common for all components:

# The library to build:
IF (RWS_USE_STATIC_LINK_COMPONENTS)
  ADD_LIBRARY(${TargetName} STATIC ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
ELSE ()
  ADD_LIBRARY(${TargetName} SHARED ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
  # Link the standard static libraries with with the shared library:
  TARGET_LINK_LIBRARIES(${TargetName} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
ENDIF ()

ADD_DEPENDENCIES(${TargetName} rwstudio)

INSTALL(TARGETS ${TargetName} DESTINATION libs)
SET(ENV{RWS_COMPONENT_LIBRARIES} "$ENV{RWS_COMPONENT_LIBRARIES}${TargetName};")
