SET(SRC_FILES 
	WorkCellBuilder.cpp
	DynamicWorkCellBuilder.cpp

	EngineTest.cpp
	FreeMotionTest.cpp
	ConstraintRevoluteTest.cpp
	CollisionBouncingBallTest.cpp
	CollisionBouncingBoxTest.cpp
	CollisionBouncingCylinderTest.cpp
	BowlFeederDropTest.cpp
	BallDropTest.cpp
	StackingBallsTest.cpp
	IntegratorRotationTest.cpp
	IntegratorSpringTest.cpp
	FrictionSlidingTest.cpp
	FrictionStaticTest.cpp
	PiHCompliantTest.cpp
	TubePlaneTest.cpp
	KVMDropTest.cpp
	KVMNutPlaneTest.cpp
	KVMTest.cpp
	AssemblyPegInTube.cpp
	MotorTest.cpp
	TwoFingerGraspTest.cpp
	
	ContactTest.cpp
	ContactPQPBoxPlaneTest.cpp
)

ADD_LIBRARY(rwsim_test ${SRC_FILES})