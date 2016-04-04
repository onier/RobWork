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

#include "BowlFeederDropTest.hpp"

#include <RobWorkConfig.hpp>
#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::trajectory;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::loaders;
using namespace rwsim::simulator;
using namespace rwsimlibs::test;

static const double DT = 0.001;

BowlFeederDropTest::BowlFeederDropTest() {
}

BowlFeederDropTest::~BowlFeederDropTest() {
}

bool BowlFeederDropTest::isEngineSupported(const std::string& engineID) const {
	if (engineID == "ODE" || engineID == "TNTIsland" || engineID == "Bullet")
		return true;
	return false;
}

void BowlFeederDropTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
	const DynamicWorkCell::Ptr dwc = getDWC(*getDefaultParameters());
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
	//engine->initPhysics(state);

	DynamicSimulator::Ptr simulator = ownedPtr(new DynamicSimulator(dwc,engine));
	simulator->init(state);

	State runState = simulator->getState();
	handle->append(TimedState(0,runState));

	double time = 0;
	double failTime = -1;
	//const long long clockBegin = TimerUtil::currentTimeMs();
	bool failed = false;
	do {
		//const long long clockBeginStep = TimerUtil::currentTimeUs();
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
		//const long long clockEndStep = TimerUtil::currentTimeUs();
		//std::cout << "step time: " << (clockEndStep-clockBeginStep) << std::endl;
		time = simulator->getTime();

		/*BOOST_FOREACH(Test* const test, _tests) {
			test->update(dt, time,runState);
		}

		if (!failed) {
			BOOST_FOREACH(Test* const test, _tests) {
				if (!test->test()) {
					_error = test->getError();
					failed = true;
					break;
				}
			}

			if (failed)
				failTime = time;
		}*/

		handle->append(TimedState(time,runState));
		handle->callback(time,failed,false);
	} while (time <= getRunTime() && !handle->isAborted());
	handle->callback(time,failed,true);
	//const long long clockEnd = rw::common::TimerUtil::currentTimeMs();

	std::stringstream errString;
	if (failed) {
		errString << "failed at time " << failTime << ": " << handle->getError();
		handle->setError(errString.str());
	}

	//const long long timeUsed = clockEnd-clockBegin;

	//BOOST_FOREACH(Test* const test, _tests) {
		//test->saveResults();
	//}

	simulator->exitPhysics();
}

double BowlFeederDropTest::getRunTime() const {
	return 1.5;
}

rw::common::Ptr<DynamicWorkCell> BowlFeederDropTest::getDWC(const PropertyMap& map) {
	if (_dwc == NULL) {
		_dwc = DynamicWorkCellLoader::load(std::string(RW_BUILD_DIR) + "Sim/test/testfiles/scene/BowlFeeder/odt0N.dwc.xml");
	}
	return _dwc;
}
