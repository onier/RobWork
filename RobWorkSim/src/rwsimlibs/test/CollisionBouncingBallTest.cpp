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

#include "CollisionBouncingBallTest.hpp"
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

#define DEFAULT_DT 0.01
#define DEFAULT_RESTITUTION 0.75

CollisionBouncingBallTest::CollisionBouncingBallTest() {
}

CollisionBouncingBallTest::~CollisionBouncingBallTest() {
}

bool CollisionBouncingBallTest::isEngineSupported(const std::string& engineID) const {
	if (engineID == "RWPhysics")
		return false;
	return true;
}

void CollisionBouncingBallTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
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
	const double dt = parameters.get("Timestep",DEFAULT_DT);
	engine->setSimulatorLog(verbose);
	engine->load(dwc);
	const ContactDetector::Ptr detector = ownedPtr(new ContactDetector(_dwc->getWorkcell()));
	if (engineID == "ODE") {
		const ContactStrategy::Ptr strat = ownedPtr(new ODEContactStrategy());
		strat->setPropertyMap(_dwc->getEngineSettings());
		detector->addContactStrategy(strat);
	} else if (engineID == "Bullet") {
		// Just let engine use own default!
	} else {
		detector->setDefaultStrategies(_dwc->getEngineSettings());
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

	const Body::Ptr body = dwc->findBody("Ball");
	RW_ASSERT(!body.isNull());
	const RigidBody::Ptr rbody = body.cast<RigidBody>();
	RW_ASSERT(!rbody.isNull());

	DynamicSimulator::Ptr simulator = ownedPtr(new DynamicSimulator(dwc,engine));
	simulator->init(state);

	State runState = simulator->getState();
	handle->append(TimedState(0,runState));

	Result result("Position in z","The z-coordinate of the object.");
	Result velocity("Velocity in z","The velocity of the object.");
	Result energy("Energy","The energy of the object.");
	const Transform3D<> Tinit = rbody->getTransformW(state);
	result.values.push_back(TimedQ(0,Q(1,Tinit.P()[2])));
	velocity.values.push_back(TimedQ(0,Q(1,rbody->getLinVelW(state)[2])));
	energy.values.push_back(TimedQ(0,Q(1,rbody->calcEnergy(state,dwc->getGravity()))));

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
		const Transform3D<> T = rbody->getTransformW(runState);
		result.values.push_back(TimedQ(time,Q(1,T.P()[2])));
		velocity.values.push_back(TimedQ(time,Q(1,rbody->getLinVelW(runState)[2])));
		energy.values.push_back(TimedQ(time,Q(1,rbody->calcEnergy(runState,dwc->getGravity()))));

		handle->append(TimedState(time,runState));
		handle->callback(time,failed,false);
	} while (time <= getRunTime() && !handle->isAborted());
	handle->append(result);
	handle->append(velocity);
	handle->append(energy);
	handle->callback(time,failed,true);

	std::stringstream errString;
	if (failed) {
		errString << "failed at time " << failTime << ": " << handle->getError();
		handle->setError(errString.str());
	}

	simulator->exitPhysics();
}

double CollisionBouncingBallTest::getRunTime() const {
	return 2.5;
}

rw::common::Ptr<DynamicWorkCell> CollisionBouncingBallTest::getDWC(const PropertyMap& map) {
	static const double radius = 0.1;
	const double restitution = map.get("Restitution",DEFAULT_RESTITUTION);
	if (_dwc == NULL) {
		const WorkCell::Ptr wc = ownedPtr(new WorkCell("CollisionBouncingBallTestWorkCell"));
		const DynamicWorkCell::Ptr dwc = ownedPtr(new DynamicWorkCell(wc));

		DynamicWorkCellBuilder builder;
		builder.addFloor(dwc);
		builder.addBall(dwc,radius,7850);
		builder.addMaterialData(dwc,0,restitution);

		const StateStructure::Ptr stateStructure = wc->getStateStructure();
		State state = stateStructure->getDefaultState();
		wc->findFrame<MovableFrame>("Ball")->setTransform(Transform3D<>(Vector3D<>::z()*0.4),state);
		stateStructure->setDefaultState(state);

		dwc->setGravity(Vector3D<>(0,0,-9.82));

		//dwc->getEngineSettings().add<double>("WorldCFM","",0);
		//dwc->getEngineSettings().add<double>("WorldERP","",0.2);

		_dwc = dwc;
	}
	return _dwc;
}

PropertyMap::Ptr CollisionBouncingBallTest::getDefaultParameters() const {
	const PropertyMap::Ptr map = EngineTest::getDefaultParameters();
	map->add("Restitution","Coefficient of restitution.",DEFAULT_RESTITUTION);
	map->add("Timestep","Timestep.",DEFAULT_DT);
	return map;
}
