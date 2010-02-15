#
#Set RobWork, RobWorkStudio and RobWorkApp ROOTs
# 
set(RW_ROOT ${ROOT}/../RobWork_branch/)
set(RWSTUDIO_ROOT ${ROOT}/../RobWorkStudio)

set(RWSIM_HAVE_RWPHYS TRUE)

#
# Bullet and Ode include,link and libs
# 
#set(BulletIncludes 
#	"c:/Program files/BULLET_PHYSICS/include"
#	${RWAPP_ROOT}/ext/bullet/Extras/GIMPACT/include
#	${RWAPP_ROOT}/ext/bullet/Extras
#	${RWAPP_ROOT}/ext/bullet/Demos
#)
#set(BulletLinkDir "c:/Program files/BULLET_PHYSICS/lib")
#set(BulletLibs 	
#    LibGIMPACTUtils 
#	LibGIMPACT
#	LibConvexDecomposition  
#	LibOpenGLSupport 
#	LibBulletDynamics 
#	LibBulletCollision 
#	LibLinearMath
#)

#SET(USE_OPENCV OFF)
#SET(OpenCV_ROOT_DIR "C:/Program Files/OpenCV/")

#SET(GRASPING_INCLUDE_DIRS "${RWAPP_ROOT}/userprojects/GraspPlanning/src")
#SET(GRASPING_LIBRARY_DIRS "${RWAPP_ROOT}/userprojects/GraspPlanning/libs/Release")
#SET(GRASPING_LIBRARIES "grasping")

SET(USE_ODE ON)
SET(ODE_INCLUDE_DIR	"C:/jimalilocal/ode-0.11.1/include")
SET(ODE_LIBRARY_DIRS "C:/jimalilocal/ode-0.11.1/lib/ReleaseSingleDll")
SET(ODE_LIBRARY "ode_single.lib")
ADD_DEFINITIONS(-DdSINGLE)
