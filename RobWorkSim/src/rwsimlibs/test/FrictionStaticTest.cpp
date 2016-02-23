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

#include "FrictionStaticTest.hpp"
#include "DynamicWorkCellBuilder.hpp"

#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsimlibs/ode/ODEContactStrategy.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::trajectory;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rwsimlibs::test;

#define DEFAULT_TILT 0.4
#define DEFAULT_FRICTION 0.5

FrictionStaticTest::FrictionStaticTest() {
}

FrictionStaticTest::~FrictionStaticTest() {
}

static const double DT = 0.01;

bool FrictionStaticTest::isEngineSupported(const std::string& engineID) const {
	if (engineID == "RWPhysics")
		return false;
	return true;
}

void FrictionStaticTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
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

	if (engineID == "ODE") {
		const ContactDetector::Ptr cd = ownedPtr(new ContactDetector(dwc->getWorkcell()));
		const ContactStrategy::Ptr strat = ownedPtr(new ODEContactStrategy());
		strat->setPropertyMap(dwc->getEngineSettings());
		cd->addContactStrategy(strat);
		RW_ASSERT(engine->setContactDetector(cd));
	//} else {
		//const ContactDetector::Ptr cd = ContactDetector::makeDefault(dwc->getWorkcell());
		//RW_ASSERT(engine->setContactDetector(cd));
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

double FrictionStaticTest::getRunTime() const {
	return 5;
}

rw::common::Ptr<DynamicWorkCell> FrictionStaticTest::getDWC(const PropertyMap& map) {
	static const double size = 0.5;
	static const double height = 0.1;

	const double tilt = map.get("Tilt",DEFAULT_TILT);
	const double friction = map.get("Friction",DEFAULT_FRICTION);

	const WorkCell::Ptr wc = ownedPtr(new WorkCell("FrictionStaticTestWorkCell"));
	const DynamicWorkCell::Ptr dwc = ownedPtr(new DynamicWorkCell(wc));
	Transform3D<> rot = Transform3D<>(RPY<>(0,-tilt,0).toRotation3D());
	//rot = Transform3D<>(EAA<>(Pi/4*rot.R().getCol(2)).toRotation3D()*rot.R());

	DynamicWorkCellBuilder builder;
	builder.addPlane(dwc,rot.R().getCol(2),0);
	builder.addBox(dwc,size,size,height,7850);
	builder.addMaterialData(dwc,friction,0);

	const StateStructure::Ptr stateStructure = wc->getStateStructure();
	State state = stateStructure->getDefaultState();
	wc->findFrame<MovableFrame>("Box")->setTransform(rot*Transform3D<>(Vector3D<>::z()*(height/2.+1e-2)),state);
	stateStructure->setDefaultState(state);

	dwc->setGravity(Vector3D<>(0,0,-9.82));

	return dwc;
}

PropertyMap::Ptr FrictionStaticTest::getDefaultParameters() const {
	const PropertyMap::Ptr map = EngineTest::getDefaultParameters();
	map->add("Tilt","Tilt in radians.",DEFAULT_TILT);
	map->add("Friction","Friction coefficient.",DEFAULT_FRICTION);
	return map;
}
