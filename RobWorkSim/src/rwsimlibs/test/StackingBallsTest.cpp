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

#include "StackingBallsTest.hpp"
#include "DynamicWorkCellBuilder.hpp"

#include <rw/geometry/Sphere.hpp>
#include <rw/geometry/Plane.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/RigidObject.hpp>
#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/dynamics/FixedBody.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>

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

static const std::size_t stackHeight = 10;
static const double DT = 0.01;

StackingBallsTest::StackingBallsTest() {
}

StackingBallsTest::~StackingBallsTest() {
	if (!(_dwc == NULL)) {
		_dwc = NULL;
		delete _dwc->getWorkcell()->findFrame("Object");
	}
}

bool StackingBallsTest::isEngineSupported(const std::string& engineID) const {
	if (engineID == "ODE" || engineID == "TNTIsland" || engineID == "Bullet")
		return true;
	return false;
}

void StackingBallsTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
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
		detector->printStrategyTable(std::cout);
	}
	State state = dwc->getWorkcell()->getDefaultState();

	DynamicSimulator::Ptr simulator = ownedPtr(new DynamicSimulator(dwc,engine));
	simulator->init(state);

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

double StackingBallsTest::getRunTime() const {
	return 1.5;
}

rw::common::Ptr<DynamicWorkCell> StackingBallsTest::getDWC(const PropertyMap& map) {
	static const double radius = 0.1;
	if (_dwc == NULL) {
		const WorkCell::Ptr wc = ownedPtr(new WorkCell("StackingBallsTestWorkCell"));
		const DynamicWorkCell::Ptr dwc = ownedPtr(new DynamicWorkCell(wc));

		DynamicWorkCellBuilder builder;
		builder.addFloor(dwc);
		for(std::size_t i = 0; i < stackHeight; i++) {
			std::stringstream name;
			name << "Ball" << i;
			builder.addBall(dwc,radius,7850,name.str());
		}
		builder.addMaterialData(dwc,0,0);

		// Set the initial state of the workcell
		const StateStructure::Ptr stateStructure = wc->getStateStructure();
		State state = stateStructure->getDefaultState();
		for(std::size_t i = 0; i < stackHeight; i++) {
			std::stringstream name;
			name << "Ball" << i;
			wc->findFrame<MovableFrame>(name.str())->setTransform(Transform3D<>(Vector3D<>::z()*radius*(1+2*i)),state);
		}
		stateStructure->setDefaultState(state);

		dwc->setGravity(Vector3D<>(0,0,-9.82));

		_dwc = dwc;
	}
	return _dwc;
}
