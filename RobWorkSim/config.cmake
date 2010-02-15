#
#Set RobWork, RobWorkStudio and RobWorkApp ROOTs
# 
set(RW_ROOT ${ROOT}/../RobWork_branch/)
set(RWSTUDIO_ROOT ${ROOT}/../RobWorkStudio)

set(RWSIM_HAVE_RWPHYS TRUE)

SET(USE_XERCES ON)
SET(XERCESC_INCLUDE_DIR "$ENV{XERCES_ROOT}/include")
SET(XERCESC_LIB_DIR "$ENV{XERCES_ROOT}/lib")

#
# Bullet and Ode include,link and libs
# 

SET(USE_ODE ON)
SET(ODE_INCLUDE_DIR	"C:/jimalilocal/ode-0.11.1/include")
SET(ODE_LIBRARY_DIRS "C:/jimalilocal/ode-0.11.1/lib/ReleaseSingleDll")
SET(ODE_LIBRARIES "ode_single.lib")
ADD_DEFINITIONS(-DdSINGLE)

SET(USE_BULLET OFF)
SET(BULLET_INCLUDE_DIR 
	"c:/Program files/BULLET_PHYSICS/include"
	${RWAPP_ROOT}/ext/bullet/Extras/GIMPACT/include
	${RWAPP_ROOT}/ext/bullet/Extras
	${RWAPP_ROOT}/ext/bullet/Demos
)	
SET(BULLET_LIBRARY_DIRS "c:/Program files/BULLET_PHYSICS/lib")
SET(BULLET_LIBRARIES 	
    LibGIMPACTUtils 
	LibGIMPACT
	LibConvexDecomposition  
	LibOpenGLSupport 
	LibBulletDynamics 
	LibBulletCollision 
	LibLinearMath
)

