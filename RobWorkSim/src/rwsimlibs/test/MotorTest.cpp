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

#include "MotorTest.hpp"
#include "DynamicWorkCellBuilder.hpp"

#include <rw/geometry/Box.hpp>
#include <rw/graphics/Model3D.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/JointDevice.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/RigidObject.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rwsim/control/PDController.hpp>
#include <rwsim/control/SpringJointController.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>

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
using namespace rwsim::simulator;
using namespace rwsimlibs::test;

#define RADIUS 0.1
#define DT 0.001

MotorTest::MotorTest() {
}

MotorTest::~MotorTest() {
}

bool MotorTest::isEngineSupported(const std::string& engineID) const {
	return true;
}

#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
using namespace rw::proximity;
using namespace rwlibs::proximitystrategies;
void MotorTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
	const bool forceControl = parameters.get<bool>("ForceControl");
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

	////
	const Transform3D<> wTballRef = Kinematics::worldTframe(dwc->getWorkcell()->findFrame("Ball"),state);
	const Transform3D<> wTbaseRef = Kinematics::worldTframe(dwc->getWorkcell()->findFrame("Device.Base"),state);
	const Transform3D<> wTfingerRef = Kinematics::worldTframe(dwc->getWorkcell()->findFrame("Device.Finger"),state);

	FKTable fk(state);
	const Transform3D<> wTball = fk.get(dwc->getWorkcell()->findFrame("Ball"));
	const Transform3D<> wTbase = fk.get(dwc->getWorkcell()->findFrame("Device.Base"));
	const Transform3D<> wTfinger = fk.get(dwc->getWorkcell()->findFrame("Device.Finger"));
	std::cout << "transforms: " << wTball << " " << wTbase << " " << wTfinger << std::endl;
	std::cout << "reference : " << wTballRef << " " << wTbaseRef << " " << wTfingerRef << std::endl;


    Transform3D<> result;
    const Frame* frame = dwc->getWorkcell()->findFrame("Device.Finger");
    const Frame* parent = frame->getParent(state);
    if (!parent) {
        result = frame->getTransform(state);
        std::cout << "no parent: " << result << std::endl;
    } else {
    	frame->multiplyTransform(fk.get(*parent), state, result);
    	std::cout << " parent: " << parent->getName() << " " << result << std::endl;
    }

	ProximityStrategyPQP::Ptr pqpStrat = ownedPtr(new ProximityStrategyPQP());
	CollisionDetector::Ptr cd = ownedPtr(new CollisionDetector(dwc->getWorkcell(),pqpStrat));
	CollisionDetector::QueryResult res;
	cd->inCollision(state, &res);
	std::cout << "MotorTest collisions: " << std::endl;
	if (res.collidingFrames.size() > 0) {
		BOOST_FOREACH(const FramePair& pair, res.collidingFrames) {
			std::cout<<"Colliding: "<<pair.first->getName()<<" -- "<<pair.second->getName()<<std::endl;
			Log::infoLog()<<"Colliding: "<<pair.first->getName()<<" -- "<<pair.second->getName()<<std::endl;
		}
	}
	/////

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
	controller->setTargetPos(Q(1,1.9*RADIUS));

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

double MotorTest::getRunTime() const {
	return 1;
}

DynamicWorkCell::Ptr MotorTest::getDWC(const PropertyMap& map) {
	const bool forceControl = map.get<bool>("ForceControl");

	const WorkCell::Ptr wc = ownedPtr(new WorkCell("MotorTestWorkCell"));
	const DynamicWorkCell::Ptr dwc = ownedPtr(new DynamicWorkCell(wc));

	DynamicWorkCellBuilder builder;
	builder.addBallFixed(dwc,RADIUS);
	builder.addBoxKin(dwc,RADIUS,RADIUS,RADIUS,7850,"Device.Base");
	builder.addMaterialData(dwc,0,0);

	Joint* const finger = new PrismaticJoint("Device.Finger",Transform3D<>());
	wc->addFrame(finger,wc->findFrame("Device.Base"));
	finger->setFixedTransform(Transform3D<>(Vector3D<>::z()*RADIUS));

	std::vector<std::pair<BodyInfo,Object::Ptr> > objects;
	{
		const DynamicWorkCellBuilder::PaHColors colors;

		GeometryData::Ptr geoData = ownedPtr(new Box(5*RADIUS,RADIUS,RADIUS/2));

		const Model3D::Material material("FingerMaterial",colors.dynamicBodies[0],colors.dynamicBodies[1],colors.dynamicBodies[2]);
		const Model3D::Ptr model = ownedPtr(new Model3D("Finger"));
		model->addTriMesh(material,*geoData->getTriMesh());

		const RigidObject::Ptr object = ownedPtr(new RigidObject(finger));
		object->addGeometry(ownedPtr(new Geometry(geoData, "Finger")));
		object->addModel(model);
		wc->add(object);

		BodyInfo info;
		builder.defaultInfo(info);
		builder.boxInfo(info,5*RADIUS,RADIUS,RADIUS/2,7850);
		objects.push_back(std::make_pair(info,object));
	}

	// Set the initial state of the workcell
	const StateStructure::Ptr stateStructure = wc->getStateStructure();
	State state = stateStructure->getDefaultState();
	wc->findFrame<MovableFrame>("Device.Base")->setTransform(Transform3D<>(Vector3D<>::x()*3*RADIUS,RPY<>(0,-Pi/2,0).toRotation3D()),state);
	//wc->findFrame<MovableFrame>("Ball")->setTransform(Transform3D<>(),state);
	stateStructure->setDefaultState(state);

	std::vector<Frame*> ends;
	ends.push_back(finger);
	const JointDevice::Ptr jdev = ownedPtr(new TreeDevice(wc->findFrame("Device.Base"),ends,"Device",state));
	jdev->setBounds(std::make_pair(Q(1,0.),Q(1,2*RADIUS)));
	jdev->setVelocityLimits(Q(1,1.));
	wc->add(jdev);

	const RigidDevice::Ptr device = ownedPtr(new RigidDevice(dwc->findBody("Device.Base"),objects,jdev));
	device->setMotorForceLimits(Q(1,25.));
	BOOST_FOREACH(const Body::Ptr link, device->getLinks()) {
		dwc->addBody(link);
	}
	dwc->addDevice(device);

	dwc->setGravity(Vector3D<>(0,0,-9.82));

	// No self-collision in gripper
	builder.contactsExclude(dwc,"Device.*","Device.*");

	if (!forceControl) {
		const SimulatedController::Ptr controller = ownedPtr(new PDController("GripperController",device,JointController::POSITION,PDParam(10,0.3),DT));
		dwc->addController(controller);
	} else {
		std::vector<SpringJointController::SpringParam> springParam(1);
		springParam[0].elasticity = map.get<double>("FingerP");
		springParam[0].dampening = map.get<double>("FingerD");
		springParam[0].offset = 0;
		const SimulatedController::Ptr controller = ownedPtr(new SpringJointController("GripperController",device,springParam));
		dwc->addController(controller);
	}

	return dwc;
}

PropertyMap::Ptr MotorTest::getDefaultParameters() const {
	const PropertyMap::Ptr map = EngineTest::getDefaultParameters();
	map->add<bool>("ForceControl","A constant force is applied if true, otherwise a constant velocity is set.",true);
	map->add<double>("FingerP","P factor.",1000);
	map->add<double>("FingerD","D factor.",350);
	return map;
}
