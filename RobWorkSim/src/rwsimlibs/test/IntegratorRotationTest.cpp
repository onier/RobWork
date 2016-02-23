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

#include "IntegratorRotationTest.hpp"
#include "FreeMotionTest.hpp"

#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::trajectory;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rwsimlibs::test;

#define DEFAULT_DT 10 // in ms
static const Vector3D<> ANGULAR_VELOCITY(0,0,4*Pi);

IntegratorRotationTest::IntegratorRotationTest(const std::string& integratorType):
	_integratorType(integratorType)
{
}

IntegratorRotationTest::~IntegratorRotationTest() {
	if (!(_dwc == NULL)) {
		_dwc = NULL;
	}
}

bool IntegratorRotationTest::isEngineSupported(const std::string& engineID) const {
	if (engineID == "RWPhysics")
		return false;
	return true;
}

void IntegratorRotationTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
	const double dt = parameters.get<double>("Timestep")/1000.;

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
	if (engineID == "TNTIsland") {
		const ContactDetector::Ptr detector = ContactDetector::makeDefault(_dwc->getWorkcell(),_dwc->getEngineSettings());
		engine->setContactDetector(detector);
	}
	State state = dwc->getWorkcell()->getDefaultState();
	const Body::Ptr body = dwc->findBody("Object");
	RW_ASSERT(!body.isNull());
	const RigidBody::Ptr rbody = body.cast<RigidBody>();
	RW_ASSERT(!rbody.isNull());
	rbody->setAngVelW(ANGULAR_VELOCITY,state);

	DynamicSimulator::Ptr simulator = ownedPtr(new DynamicSimulator(dwc,engine));
	simulator->init(state);

	State runState = simulator->getState();
	handle->append(TimedState(0,runState));

	Result result("Energy","The energy of the object.");
	Result velocity("Angular Velocity","The size of the angular velocity.");
	result.values.push_back(TimedQ(0,Q(1,rbody->calcEnergy(state))));
	velocity.values.push_back(TimedQ(0,Q(1,rbody->getAngVel(state).norm2())));

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
		result.values.push_back(TimedQ(time,Q(1,rbody->calcEnergy(runState))));
		velocity.values.push_back(TimedQ(time,Q(1,rbody->getAngVel(runState).norm2())));

		handle->append(TimedState(time,runState));
		handle->callback(time,failed,false);
	} while (time <= getRunTime() && !handle->isAborted());
	handle->append(result);
	handle->append(velocity);
	handle->callback(time,failed,true);

	std::stringstream errString;
	if (failed) {
		errString << "failed at time " << failTime << ": " << handle->getError();
		handle->setError(errString.str());
	}

	simulator->exitPhysics();
}

double IntegratorRotationTest::getRunTime() const {
	return 30;
}

rw::common::Ptr<DynamicWorkCell> IntegratorRotationTest::getDWC(const PropertyMap& map) {
	if (_dwc == NULL) {
		_dwc = FreeMotionTest::makeDWC(_integratorType);
		_dwc->setGravity(Vector3D<>(0,0,0));
	}
	return _dwc;
}

PropertyMap::Ptr IntegratorRotationTest::getDefaultParameters() const {
	const PropertyMap::Ptr map = EngineTest::getDefaultParameters();
	map->add<double>("Timestep","Default timestep size in ms.",DEFAULT_DT);
	return map;
}

double IntegratorRotationTest::getExpectedEnergy(const DynamicWorkCell::Ptr dwc) const {
	State state = dwc->getWorkcell()->getDefaultState();
	const Body::Ptr body = dwc->findBody("Object");
	RW_ASSERT(!body.isNull());
	const RigidBody::Ptr rbody = body.cast<RigidBody>();
	RW_ASSERT(!rbody.isNull());
	rbody->setAngVelW(ANGULAR_VELOCITY,state);
	return rbody->calcEnergy(state);
}
