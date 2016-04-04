/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "CollisionBouncingBoxTest.hpp"
#include "DynamicWorkCellBuilder.hpp"

#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/contacts/ContactStrategyPQP.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsimlibs/ode/ODEContactStrategy.hpp>
#include <rwsimlibs/bullet/BtContactStrategy.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::trajectory;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rwsimlibs::bullet;
using namespace rwsimlibs::test;

#define DEFAULT_DT 0.025
#define DEFAULT_HEIGHT 0.2
#define DEFAULT_RESTITUTION 0.75
#define DEFAULT_TILT_X -0.1
#define DEFAULT_TILT_Y -0.1

CollisionBouncingBoxTest::CollisionBouncingBoxTest() {
}

CollisionBouncingBoxTest::~CollisionBouncingBoxTest() {
}

bool CollisionBouncingBoxTest::isEngineSupported(const std::string& engineID) const {
	if (engineID == "RWPhysics")
		return false;
	return true;
}

void CollisionBouncingBoxTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
	const double dt = parameters.get("Timestep",DEFAULT_DT);
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
	const ContactDetector::Ptr detector = ownedPtr(new ContactDetector(dwc->getWorkcell()));
	if (engineID == "ODE") {
		const ContactStrategy::Ptr strat = ownedPtr(new ODEContactStrategy());
		strat->setPropertyMap(dwc->getEngineSettings());
		detector->addContactStrategy(strat);
	} else if (engineID == "Bullet") {
		// Just let engine use own default!
	} else {
		detector->setDefaultStrategies(dwc->getEngineSettings());
	}
	if (engineID != "Bullet") {
		if (!engine->setContactDetector(detector)) {
			handle->setError("Engine did not accept a ContactDetector.");
			return;
		}
		std::cout << "set CD: " << std::endl;
		detector->printStrategyTable(std::cout);
	}
	State state = dwc->getWorkcell()->getDefaultState();
	//engine->initPhysics(state);

	DynamicSimulator::Ptr simulator = ownedPtr(new DynamicSimulator(dwc,engine));
	simulator->init(state);

	State runState = simulator->getState();
	handle->append(TimedState(0,runState));

	double time = 0;
	double failTime = -1;
	bool failed = false;
	do {
		try {
			simulator->step(dt);
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

double CollisionBouncingBoxTest::getRunTime() const {
	return 2.5;
}

rw::common::Ptr<DynamicWorkCell> CollisionBouncingBoxTest::getDWC(const PropertyMap& map) {
	static const double x = 0.5;
	static const double y = 0.25;
	static const double z = 0.1;
	const double restitution = map.get("Restitution",DEFAULT_RESTITUTION);
	const double tiltX = map.get("Tilt X",DEFAULT_TILT_X);
	const double tiltY = map.get("Tilt Y",DEFAULT_TILT_Y);
	const double height = map.get("Height",DEFAULT_HEIGHT);
	const bool trimesh = map.get("TriMesh",false);
	const bool correction = map.get("Enable Correction",true);

	const WorkCell::Ptr wc = ownedPtr(new WorkCell("CollisionBouncingBallTestWorkCell"));
	const DynamicWorkCell::Ptr dwc = ownedPtr(new DynamicWorkCell(wc));

	DynamicWorkCellBuilder builder;
	builder.addFloor(dwc,"Floor",trimesh);
	builder.addBox(dwc,x,y,z,7850,"Box",trimesh);
	builder.addMaterialData(dwc,0,restitution);

	const StateStructure::Ptr stateStructure = wc->getStateStructure();
	State state = stateStructure->getDefaultState();
	const Rotation3D<> rot = EAA<>(tiltX,tiltY,0).toRotation3D();
	wc->findFrame<MovableFrame>("Box")->setTransform(Transform3D<>(Vector3D<>::z()*height,rot),state);
	stateStructure->setDefaultState(state);

	dwc->setGravity(Vector3D<>(0,0,-9.82));
	dwc->getEngineSettings().add<int>("TNTCorrection","",correction?1:0);
	dwc->getEngineSettings().add<double>("TNTCorrectionContactLayer","",0.1e-3);

	return dwc;
}

PropertyMap::Ptr CollisionBouncingBoxTest::getDefaultParameters() const {
	const PropertyMap::Ptr map = EngineTest::getDefaultParameters();
	map->add("Height","Height of box (in meters).",DEFAULT_HEIGHT);
	map->add("Tilt X","Rotation about x-axis (in radians).",DEFAULT_TILT_X);
	map->add("Tilt Y","Rotation about y-axis (in radians).",DEFAULT_TILT_Y);
	map->add("Restitution","Coefficient of restitution.",DEFAULT_RESTITUTION);
	map->add("TriMesh","Use a trimesh representation (for contact detection with PQP strategy).",false);
	map->add("Timestep","Decrease if trimesh representation is used to get stability).",DEFAULT_DT);
	map->add("Enable Correction","Used for trimesh representation for debugging).",true);
	return map;
}
