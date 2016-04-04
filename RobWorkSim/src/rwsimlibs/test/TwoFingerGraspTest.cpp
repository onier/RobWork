/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "TwoFingerGraspTest.hpp"
#include "DynamicWorkCellBuilder.hpp"

#include <rw/geometry/Box.hpp>
#include <rw/graphics/Model3D.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/RigidObject.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/DependentPrismaticJoint.hpp>
#include <rwsim/control/PDController.hpp>
#include <rwsim/control/SpringJointController.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::trajectory;
using namespace rwlibs::control;
using namespace rwlibs::simulation;
using namespace rwsim::control;
using namespace rwsim::dynamics;
using namespace rwsim::log;
using namespace rwsim::simulator;
using namespace rwsimlibs::test;

#define radius 0.1
#define DT 0.001

TwoFingerGraspTest::TwoFingerGraspTest() {
}

TwoFingerGraspTest::~TwoFingerGraspTest() {
}

bool TwoFingerGraspTest::isEngineSupported(const std::string& engineID) const {
	return true;
}

void TwoFingerGraspTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<SimulatorLogScope> verbose) {
	const bool forceControl = parameters.get<bool>("ForceControl");
	const bool coupled = parameters.get<bool>("Coupled");
	const DynamicWorkCell::Ptr dwc = getDWC(parameters);
	if (dwc == NULL) {
		handle->setError("Could not make engine.");
		return;
	}
	const PhysicsEngine::Ptr engine = PhysicsEngine::Factory::makePhysicsEngine(engineID);
	if (engine == NULL) {
		handle->setError("Could not make engine.");
		return;
	}
	engine->setSimulatorLog(verbose);
	engine->load(dwc);
	State state = dwc->getWorkcell()->getDefaultState();

	handle->callback(0,false,false);
	DynamicSimulator::Ptr simulator = ownedPtr(new DynamicSimulator(dwc,engine));
	try {
		simulator->init(state);
	} catch(const Exception& e) {
		handle->setError(e.what());
		handle->callback(0,true,true);
		return;
	}

	JointController::Ptr controller;
	if (!forceControl) {
		controller = dwc->findController("GripperController").cast<PDController>();
	} else {
		controller = dwc->findController("GripperController").cast<SpringJointController>();
	}
	if (coupled) {
		controller->setTargetPos(Q(1,1.9*radius));
	} else {
		controller->setTargetPos(Q(2,1.9*radius,1.9*radius));
	}

	State runState = simulator->getState();
	handle->append(TimedState(0,runState));

	double time = 0;
	double failTime = -1;
	bool failed = false;
	do {
		try {
			simulator->step(DT);
			runState = simulator->getState();
		} catch(const Exception& e) {
			failed = true;
			failTime = simulator->getTime();
			handle->setError(e.what());
			break;
		} catch(...) {
			failed = true;
			failTime = simulator->getTime();
			handle->setError("unknown exception!");
			break;
		}
		time = simulator->getTime();

		handle->append(TimedState(time,runState));
		handle->callback(time,failed,false);
	} while (time <= getRunTime() && !handle->isAborted());
	handle->callback(time,failed,true);

	std::stringstream errString;
	if (failed) {
		errString << "failed at time " << failTime << ": " << handle->getError();
		handle->setError(errString.str());
	}

	simulator->exitPhysics();
}

double TwoFingerGraspTest::getRunTime() const {
	return 1.5;
}

DynamicWorkCell::Ptr TwoFingerGraspTest::getDWC(const PropertyMap& map) {
	const bool forceControl = map.get<bool>("ForceControl");
	const bool kinematicBase = map.get<bool>("KinematicBase");
	const bool graspCylinder = map.get<bool>("GraspCylinder");
	const bool coupled = map.get<bool>("Coupled");

	const static double EPS = 1e-3;
	const WorkCell::Ptr wc = ownedPtr(new WorkCell("TwoFingerGraspTestWorkCell"));
	const DynamicWorkCell::Ptr dwc = ownedPtr(new DynamicWorkCell(wc));

	DynamicWorkCellBuilder builder;
	builder.addFloor(dwc);
	if (graspCylinder)
		builder.addCylinder(dwc,radius,radius*5,7850);
	if (kinematicBase) {
		builder.addBoxKin(dwc,5*radius,radius,2*radius,7850,"GripperDevice.Base");
	} else {
		builder.addBox(dwc,5*radius,radius,2*radius,7850,"GripperDevice.Base");
		builder.addBoxKin(dwc,radius/2,radius,2*radius,7850,"Control");
	}
	builder.addMaterialData(dwc,0,0);

	Joint* const left = new PrismaticJoint("GripperDevice.Left",Transform3D<>());
	Joint* right;
	if (coupled) {
		right = new DependentPrismaticJoint("GripperDevice.Right",Transform3D<>(),left,1,0);
	} else {
		right = new PrismaticJoint("GripperDevice.Right",Transform3D<>());
	}
	wc->addFrame(left,wc->findFrame("GripperDevice.Base"));
	wc->addFrame(right,wc->findFrame("GripperDevice.Base"));
	left->setFixedTransform(Transform3D<>(-Vector3D<>::x()*2.25*radius-Vector3D<>::z()*3.5*radius,RPY<>(0,Pi/2,0).toRotation3D()));
	right->setFixedTransform(Transform3D<>(Vector3D<>::x()*2.25*radius-Vector3D<>::z()*3.5*radius,RPY<>(0,-Pi/2,0).toRotation3D()));

	std::vector<std::pair<BodyInfo,Object::Ptr> > objects;
	{
		const DynamicWorkCellBuilder::PaHColors colors;

		GeometryData::Ptr geoData = ownedPtr(new Box(5*radius,radius,radius/2));

		const Model3D::Material material("FingerMaterial",colors.dynamicBodies[0],colors.dynamicBodies[1],colors.dynamicBodies[2]);
		const Model3D::Ptr model = ownedPtr(new Model3D("Finger"));
		model->addTriMesh(material,*geoData->getTriMesh());

		const RigidObject::Ptr lobject = ownedPtr(new RigidObject(left));
		const RigidObject::Ptr robject = ownedPtr(new RigidObject(right));
		lobject->addGeometry(ownedPtr(new Geometry(geoData, "Finger")));
		robject->addGeometry(ownedPtr(new Geometry(geoData, "Finger")));
		lobject->addModel(model);
		robject->addModel(model);
		wc->add(lobject);
		wc->add(robject);

		BodyInfo info;
		builder.defaultInfo(info);
		builder.boxInfo(info,5*radius,radius,radius/2,7850);
		objects.push_back(std::make_pair(info,lobject));
		objects.push_back(std::make_pair(info,robject));
	}

	// Set the initial state of the workcell
	const StateStructure::Ptr stateStructure = wc->getStateStructure();
	State state = stateStructure->getDefaultState();
	wc->findFrame<MovableFrame>("GripperDevice.Base")->setTransform(Transform3D<>(Vector3D<>::z()*6.75*radius,RPY<>(Pi/2,0,0).toRotation3D()),state);
	if (graspCylinder)
		wc->findFrame<MovableFrame>("Cylinder")->setTransform(Transform3D<>(Vector3D<>::z()*(radius+EPS),RPY<>(0,Pi/2,0).toRotation3D()),state);
	if (!kinematicBase)
		wc->findFrame<MovableFrame>("Control")->setTransform(Transform3D<>(Vector3D<>::z()*10*radius,RPY<>(Pi/2,0,0).toRotation3D()),state);
	stateStructure->setDefaultState(state);

	std::vector<Frame*> ends;
	ends.push_back(left);
	ends.push_back(right);
	const JointDevice::Ptr jdev = ownedPtr(new TreeDevice(wc->findFrame("GripperDevice.Base"),ends,"GripperDevice",state));
	if (coupled) {
		jdev->setBounds(std::make_pair(Q(1,0.),Q(1,2*radius)));
		jdev->setVelocityLimits(Q(1,1.));
	} else {
		jdev->setBounds(std::make_pair(Q(2,0,0),Q(2,2*radius,2*radius)));
		jdev->setVelocityLimits(Q(2,1.,1.));
	}
	wc->add(jdev);

	const RigidDevice::Ptr device = ownedPtr(new RigidDevice(dwc->findBody("GripperDevice.Base"),objects,jdev));
	if (coupled)
		device->setMotorForceLimits(Q(1,25.));
	else
		device->setMotorForceLimits(Q(2,25.,25.));
	BOOST_FOREACH(const Body::Ptr link, device->getLinks()) {
		dwc->addBody(link);
	}
	dwc->addDevice(device);

	if (!kinematicBase) {
		RW_ASSERT(!dwc->findBody("Control").isNull());
		RW_ASSERT(!dwc->findBody("GripperDevice.Base").isNull());
		const Constraint::Ptr c = ownedPtr(new Constraint("ControlSpring",Constraint::Free,dwc->findBody("Control").get(),dwc->findBody("GripperDevice.Base").get()));
		Constraint::SpringParams spring;
		spring.enabled = true;
		spring.compliance = Eigen::MatrixXd::Zero(6,6);
		spring.damping = Eigen::MatrixXd::Zero(6,6);
		for (Eigen::MatrixXd::Index i = 0; i < 3; i++) {
			spring.compliance(i,i) = 1./map.get<double>("BaseKLin");
			spring.damping(i,i) = map.get<double>("BaseDampingLin");
		}
		for (Eigen::MatrixXd::Index i = 3; i < 6; i++) {
			spring.compliance(i,i) = 1./map.get<double>("BaseKAng");
			spring.damping(i,i) = map.get<double>("BaseDampingAng");
		}
		c->setSpringParams(spring);
		c->setTransform(Transform3D<>(-Vector3D<>::z()*3.25*radius));
		dwc->addConstraint(c);
	}

	dwc->setGravity(Vector3D<>(0,0,-9.82));

	// No self-collision in gripper
	builder.contactsExclude(dwc,"GripperDevice.*","GripperDevice.*");
	if (!kinematicBase) {
		// Kinematic control frame can not collide with anything.
		builder.contactsExclude(dwc,"Control","*");
	} else {
		// We do not move gripper into floor
		builder.contactsExclude(dwc,"Floor","GripperDevice.*");
	}

	if (!forceControl) {
		const SimulatedController::Ptr controller = ownedPtr(new PDController("GripperController",device,JointController::POSITION,PDParam(10,0.3),DT));
		dwc->addController(controller);
	} else {
		std::vector<SpringJointController::SpringParam> springParam(1);
		springParam[0].elasticity = map.get<double>("FingerP");
		springParam[0].dampening = map.get<double>("FingerD");
		springParam[0].offset = 0;
		if (!coupled) {
			springParam.resize(2);
			springParam[1].elasticity = springParam[0].elasticity;
			springParam[1].dampening = springParam[0].dampening;
			springParam[1].offset = springParam[0].offset;
		}
		const SimulatedController::Ptr controller = ownedPtr(new SpringJointController("GripperController",device,springParam));
		dwc->addController(controller);
	}

	dwc->getEngineSettings().add<std::string>("RWPECollisionSolver","","Simultaneous");

	return dwc;
}

PropertyMap::Ptr TwoFingerGraspTest::getDefaultParameters() const {
	const PropertyMap::Ptr map = EngineTest::getDefaultParameters();
	map->add<bool>("ForceControl","A constant force is applied if true, otherwise a constant velocity is set.",true);
	map->add<bool>("GraspCylinder","Grasp a cylinder.",true);
	map->add<bool>("KinematicBase","Use a kinematic base and no spring.",false);
	map->add<bool>("Coupled","Use coupled fingers.",true);
	map->add<double>("FingerP","P factor.",1000);
	map->add<double>("FingerD","D factor.",350);
	map->add<double>("BaseKLin","",50000);
	map->add<double>("BaseDampingLin","",1000);
	map->add<double>("BaseKAng","",1);
	map->add<double>("BaseDampingAng","",0.1);
	return map;
}
