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

#include "CollisionBouncingCylinderTest.hpp"
#include "DynamicWorkCellBuilder.hpp"

#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/contacts/ContactStrategyPQP.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsimlibs/ode/ODEContactStrategy.hpp>

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
using namespace rwsimlibs::test;

static const double DT = 0.01;

CollisionBouncingCylinderTest::CollisionBouncingCylinderTest() {
}

CollisionBouncingCylinderTest::~CollisionBouncingCylinderTest() {
}

bool CollisionBouncingCylinderTest::isEngineSupported(const std::string& engineID) const {
	if (engineID == "RWPhysics")
		return false;
	return true;
}

void CollisionBouncingCylinderTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
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
	const ContactDetector::Ptr detector = ownedPtr(new ContactDetector(_dwc->getWorkcell()));
	if (engineID == "ODE") {
		const ContactStrategy::Ptr strat = ownedPtr(new ODEContactStrategy());
		strat->setPropertyMap(_dwc->getEngineSettings());
		detector->addContactStrategy(strat);
	} else {
		detector->setDefaultStrategies(_dwc->getEngineSettings());
	}
	if (engineID != "Bullet") {
		if (!engine->setContactDetector(detector)) {
			handle->setError("Engine did not accept a ContactDetector.");
			return;
		}
	}
	detector->printStrategyTable(std::cout);
	State state = dwc->getWorkcell()->getDefaultState();

	const Body::Ptr body = dwc->findBody("Cylinder");
	RW_ASSERT(!body.isNull());
	const RigidBody::Ptr rbody = body.cast<RigidBody>();
	RW_ASSERT(!rbody.isNull());

	DynamicSimulator::Ptr simulator = ownedPtr(new DynamicSimulator(dwc,engine));
	simulator->init(state);

	State runState = simulator->getState();
	handle->append(TimedState(0,runState));

	Result result("Position in z","The z-coordinate of the object.");
	Result velocityLin("Linear Velocity","The linear velocity of the object.");
	Result velocityAng("Angular Velocity","The angular velocity of the object.");
	Result energy("Energy","The energy of the object.");
	const Transform3D<> Tinit = rbody->getTransformW(state);
	result.values.push_back(TimedQ(0,Q(1,Tinit.P()[2])));
	velocityLin.values.push_back(TimedQ(0,Q(1,rbody->getLinVel(state).norm2())));
	velocityAng.values.push_back(TimedQ(0,Q(1,rbody->getAngVel(state).norm2())));
	energy.values.push_back(TimedQ(0,Q(1,rbody->calcEnergy(state,dwc->getGravity()))));

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
		const Transform3D<> T = rbody->getTransformW(runState);
		result.values.push_back(TimedQ(time,Q(1,T.P()[2])));
		velocityLin.values.push_back(TimedQ(time,Q(1,rbody->getLinVel(runState).norm2())));
		velocityAng.values.push_back(TimedQ(time,Q(1,rbody->getAngVel(runState).norm2())));
		energy.values.push_back(TimedQ(time,Q(1,rbody->calcEnergy(runState,dwc->getGravity()))));

		handle->append(TimedState(time,runState));
		handle->callback(time,failed,false);
	} while (time <= getRunTime() && !handle->isAborted());
	handle->append(result);
	handle->append(velocityLin);
	handle->append(velocityAng);
	handle->append(energy);
	handle->callback(time,failed,true);

	std::stringstream errString;
	if (failed) {
		errString << "failed at time " << failTime << ": " << handle->getError();
		handle->setError(errString.str());
	}

	simulator->exitPhysics();
}

double CollisionBouncingCylinderTest::getRunTime() const {
	return 0.5;
}

rw::common::Ptr<DynamicWorkCell> CollisionBouncingCylinderTest::getDWC(const PropertyMap& map) {
	static const double radius = 0.05;
	static const double length = 0.25;
	static const double tilt = Pi/4;
	if (_dwc == NULL) {
		const WorkCell::Ptr wc = ownedPtr(new WorkCell("CollisionBouncingCylinderTestWorkCell"));
		const DynamicWorkCell::Ptr dwc = ownedPtr(new DynamicWorkCell(wc));

		DynamicWorkCellBuilder builder;
		builder.addFloor(dwc);
		builder.addCylinder(dwc,radius,length,7850);
		builder.addMaterialData(dwc,0,getRestitution());

		const StateStructure::Ptr stateStructure = wc->getStateStructure();
		State state = stateStructure->getDefaultState();
		wc->findFrame<MovableFrame>("Cylinder")->setTransform(Transform3D<>(Vector3D<>::z()*0.25,RPY<>(0,tilt,0)),state);
		stateStructure->setDefaultState(state);

		dwc->setGravity(Vector3D<>(0,0,-9.82));

		dwc->getEngineSettings().add<double>("WorldCFM","",0);
		dwc->getEngineSettings().add<double>("WorldERP","",0.2);

		_dwc = dwc;
	}
	return _dwc;
}

double CollisionBouncingCylinderTest::getRestitution() const {
	return 0.25;
}
