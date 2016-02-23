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

#include "ConstraintRevoluteTest.hpp"
#include "DynamicWorkCellBuilder.hpp"

#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/sensor/SimulatedFTSensor.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::trajectory;
using namespace rwsim::dynamics;
using namespace rwsim::sensor;
using namespace rwsim::simulator;
using namespace rwsimlibs::test;

#define DEFAULT_DT 0.01
#define DEFAULT_RUNTIME 10
#define RADIUS 0.1

ConstraintRevoluteTest::ConstraintRevoluteTest() {
}

ConstraintRevoluteTest::~ConstraintRevoluteTest() {
}

bool ConstraintRevoluteTest::isEngineSupported(const std::string& engineID) const {
	return true;
}

void ConstraintRevoluteTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
	static const double MASS = 7850.*4/3*Pi*RADIUS*RADIUS*RADIUS;
	static const double ENERGY_REF = MASS*9.82*0.4;

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
	const double dt = parameters.get("Timestep",DEFAULT_DT)/1000.;
	engine->setSimulatorLog(verbose);
	engine->load(dwc);
	State state = dwc->getWorkcell()->getDefaultState();

	const Body::Ptr body = dwc->findBody("Ball");
	RW_ASSERT(!body.isNull());
	const RigidBody::Ptr rbody = body.cast<RigidBody>();
	RW_ASSERT(!rbody.isNull());
	const SimulatedFTSensor::Ptr sensor = _dwc->findSensor("ConstraintForceSensor").cast<SimulatedFTSensor>();
	RW_ASSERT(!sensor.isNull());

	DynamicSimulator::Ptr simulator = ownedPtr(new DynamicSimulator(dwc,engine));
	simulator->init(state);

	State runState = simulator->getState();
	handle->append(TimedState(0,runState));

	Result result("Position in z","The z-coordinate of the object.");
	Result velocityLin("Linear Velocity","The linear velocity of the object.");
	Result velocityAng("Angular Velocity","The angular velocity of the object.");
	Result energy("Energy","The energy of the object.");
	Result force("Force","The size of the constraint force.");
	const Transform3D<> Tinit = rbody->getTransformW(state);
	result.values.push_back(TimedQ(0,Q(1,Tinit.P()[2])));
	velocityLin.values.push_back(TimedQ(0,Q(1,rbody->getLinVelW(state).norm2())));
	velocityAng.values.push_back(TimedQ(0,Q(1,rbody->getAngVelW(state).norm2())));
	energy.values.push_back(TimedQ(0,Q(1,rbody->calcEnergy(state,dwc->getGravity()))));
	force.values.push_back(TimedQ(0,Q(1,0.)));

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
		velocityLin.values.push_back(TimedQ(time,Q(1,rbody->getLinVelW(runState).norm2())));
		velocityAng.values.push_back(TimedQ(time,Q(1,rbody->getAngVelW(runState).norm2())));
		energy.values.push_back(TimedQ(time,Q(1,rbody->calcEnergy(runState,dwc->getGravity()))));
		force.values.push_back(TimedQ(time,Q(1,sensor->getForce(runState).norm2())));

		handle->append(TimedState(time,runState));

		if (energy.values.back().getValue()[0] > ENERGY_REF*1.01) {
			failed = true;
			failTime= time;
			std::stringstream errString;
			errString << "Reference energy level exceeded more than 1% (" << energy.values.back().getValue()[0] << " > " << ENERGY_REF*1.01 << ")";
			handle->setError(errString.str());
			break;
		}

		handle->callback(time,failed,false);
	} while (time <= getRunTime() && !handle->isAborted());
	handle->append(result);
	handle->append(velocityLin);
	handle->append(velocityAng);
	handle->append(energy);
	handle->append(force);

	std::stringstream errString;
	if (failed) {
		errString << "Failed at time " << failTime << ": " << handle->getError();
		handle->setError(errString.str());
	}

	handle->callback(time,failed,true);

	simulator->exitPhysics();
}

double ConstraintRevoluteTest::getRunTime() const {
	return DEFAULT_RUNTIME;
}

DynamicWorkCell::Ptr ConstraintRevoluteTest::getDWC(const PropertyMap& map) {
	if (_dwc.isNull()) {
		const WorkCell::Ptr wc = ownedPtr(new WorkCell("ConstraintRevoluteTestWorkCell"));
		const DynamicWorkCell::Ptr dwc = ownedPtr(new DynamicWorkCell(wc));

		DynamicWorkCellBuilder builder;
		builder.addFloor(dwc);
		builder.addBoxKin(dwc,RADIUS,RADIUS,RADIUS,-1);
		builder.addBall(dwc,RADIUS,7850,"Ball","Box");
		builder.addMaterialData(dwc,0,0);

		const StateStructure::Ptr stateStructure = wc->getStateStructure();
		State state = stateStructure->getDefaultState();
		wc->findFrame<MovableFrame>("Box")->setTransform(Transform3D<>(Vector3D<>::z()*0.4,EAA<>(-Pi/2.,0,0)),state);
		wc->findFrame<MovableFrame>("Ball")->setTransform(Transform3D<>(Vector3D<>::x()*0.2),state);
		stateStructure->setDefaultState(state);

		dwc->setGravity(Vector3D<>(0,0,-9.82));

		const Body::Ptr fixedBody = dwc->findBody("Box");
		const Body::Ptr movingBody = dwc->findBody("Ball");
		const Constraint::Ptr constraint = ownedPtr(new Constraint("Constraint",Constraint::Revolute,fixedBody.get(),movingBody.get()));
		dwc->addConstraint(constraint);

		// Finally add sensors
		const SimulatedFTSensor::Ptr sensor = ownedPtr(new SimulatedFTSensor("ConstraintForceSensor",fixedBody,movingBody));
		dwc->addSensor(sensor);

		_dwc = dwc;
	}
	return _dwc;
}

PropertyMap::Ptr ConstraintRevoluteTest::getDefaultParameters() const {
	const PropertyMap::Ptr map = EngineTest::getDefaultParameters();
	map->add("Timestep","Timestep in milliseconds.",DEFAULT_DT*1000);
	return map;
}
