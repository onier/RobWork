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

#include "TubePlaneTest.hpp"
#include "DynamicWorkCellBuilder.hpp"

#include <rw/geometry/Tube.hpp>
#include <rw/geometry/Plane.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::trajectory;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rwsimlibs::test;

#define DT 0.01
#define DEFAULT_TILT Pi/4
#define DEFAULT_HEIGHT 0.01

TubePlaneTest::TubePlaneTest() {
}

TubePlaneTest::~TubePlaneTest() {
}

bool TubePlaneTest::isEngineSupported(const std::string& engineID) const {
	return true;
}

void TubePlaneTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, const rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
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
		const ContactDetector::Ptr detector = ContactDetector::makeDefault(dwc->getWorkcell(),dwc->getEngineSettings());
		engine->setContactDetector(detector);
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

double TubePlaneTest::getRunTime() const {
	return 2.0;
}

rw::common::Ptr<DynamicWorkCell> TubePlaneTest::getDWC(const PropertyMap& map) {
	static const double radius = 0.025;
	static const double thickness = 0.005;
	static const double length = 0.1;
	const double tilt = map.get("Tilt",DEFAULT_TILT);
	const double height = map.get("Height",DEFAULT_HEIGHT);
	const WorkCell::Ptr wc = ownedPtr(new WorkCell("TubePlaneTestWorkCell"));
	const DynamicWorkCell::Ptr dwc = ownedPtr(new DynamicWorkCell(wc));

	DynamicWorkCellBuilder builder;
	builder.addFloor(dwc);
	builder.addTube(dwc,radius,thickness,length,7850);
	builder.addMaterialData(dwc);

	const StateStructure::Ptr stateStructure = wc->getStateStructure();
	State state = stateStructure->getDefaultState();
	wc->findFrame<MovableFrame>("Tube")->setTransform(Transform3D<>(Vector3D<>::z()*(length/2.+height),RPY<>(0,tilt,0).toRotation3D()),state);
	stateStructure->setDefaultState(state);

	dwc->setGravity(Vector3D<>(0,0,-9.82));

	return dwc;
}

PropertyMap::Ptr TubePlaneTest::getDefaultParameters() const {
	const PropertyMap::Ptr map = EngineTest::getDefaultParameters();
	map->add("Tilt","Amount of tilting (in radians).",DEFAULT_TILT);
	map->add("Height","Height above ground.",DEFAULT_HEIGHT);
	return map;
}
