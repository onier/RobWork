#
#Set RobWork, RobWorkStudio and RobWorkApp ROOTs
# 
set(RW_ROOT ${ROOT}/../RobWork_branch/)
set(RWSTUDIO_ROOT ${ROOT}/../RobWorkStudio)

set(RWSIM_USE_RWPHYS ON)
SET(RWSIM_USE_LUA ON)

SET(RWSIM_USE_ODE ON)
SET(ODE_DIR "C:/jimali/opende/")

SET(ODE_USE_DOUBLE ON)
#SET(ODE_USE_DEBUG ON)
#SET(ODE_USE_SINGLE ON)
#SET(ODE_LIBRARY_NAME ) # suggest specific library name
#SET(ODE_LIBRARY_DIR ) # suggest library location
#SET(ODE_LIBRARY_DIRS "C:/jimali/opende/lib/ReleaseDoubleLib")
#SET(ODE_LIBRARIES "ode_double.lib")
#ADD_DEFINITIONS(-DdDOUBLE)

SET(RWSIM_USE_BULLET OFF)
SET(BULLET_ROOT "c:/Program files/BULLET_PHYSICS/") 

#SET(BULLET_INCLUDE_DIR 
#	"c:/Program files/BULLET_PHYSICS/include"
#	${RWAPP_ROOT}/ext/bullet/Extras/GIMPACT/include
#	${RWAPP_ROOT}/ext/bullet/Extras
#	${RWAPP_ROOT}/ext/bullet/Demos
#)
