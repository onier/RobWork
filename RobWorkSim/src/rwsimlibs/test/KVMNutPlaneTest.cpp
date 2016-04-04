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

#include "KVMNutPlaneTest.hpp"

#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/contacts/ContactStrategyPQP.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
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
using namespace rwsim::loaders;
using namespace rwsim::simulator;
using namespace rwsimlibs::bullet;
using namespace rwsimlibs::test;

#define DT 0.01
#define DEFAULT_HEIGHT 0
#define DEFAULT_RESTITUTION 0
#define DEFAULT_MAXSEPDISTANCE 0.5
#define DEFAULT_PQP_UPDATE_THRESHOLD 1
#define DEFAULT_PQP_UPDATE_THRESHOLD_LIN 2.
#define DEFAULT_PQP_UPDATE_THRESHOLD_ANG 0.25

KVMNutPlaneTest::KVMNutPlaneTest() {
}

KVMNutPlaneTest::~KVMNutPlaneTest() {
}

bool KVMNutPlaneTest::isEngineSupported(const std::string& engineID) const {
	if (engineID == "RWPhysics")
		return false;
	return true;
}

void KVMNutPlaneTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
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

double KVMNutPlaneTest::getRunTime() const {
	return 2.5;
}

rw::common::Ptr<DynamicWorkCell> KVMNutPlaneTest::getDWC(const PropertyMap& map) {
	//const double restitution = map.get("Restitution",DEFAULT_RESTITUTION);
	//const double height = map.get("Height",DEFAULT_HEIGHT);
	if (_dwc.isNull()) {
		_dwc = DynamicWorkCellLoader::load("/home/thomas/Eclipse/RobWork/RobWorkSim/test/testfiles/scene/KVM/nut_plane.dwc.xml");
		_defaultSettings = _dwc->getEngineSettings();
	}
	PropertyMap& settings = _dwc->getEngineSettings();
	settings = _defaultSettings;
	const double dist = map.get<double>("MaxSepDistance")/1000.;
	const double pqp_threshold = map.get<double>("ContactStrategyPQPUpdateThresholdAbsolute")/1000.;
	const double pqp_threshold_lin = map.get<double>("ContactStrategyPQPUpdateThresholdLinear");
	const double pqp_threshold_ang = map.get<double>("ContactStrategyPQPUpdateThresholdAngular");
	const bool correction = map.get("Enable Correction",true);
	settings.set<double>("MaxSepDistance",dist);
	settings.set<double>("ContactStrategyPQPUpdateThresholdAbsolute",pqp_threshold);
	settings.set<double>("ContactStrategyPQPUpdateThresholdLinear",pqp_threshold_lin);
	settings.set<double>("ContactStrategyPQPUpdateThresholdAngular",pqp_threshold_ang);
	settings.set<int>("TNTCorrection",correction?1:0);

	std::cout << "Nut inertia: " << std::endl;
	std::cout << _dwc->findBody("Nut")->getInertia() << std::endl;
	std::cout << "Nut com: " << _dwc->findBody("Nut")->getInfo().masscenter << std::endl;

	return _dwc;
}

PropertyMap::Ptr KVMNutPlaneTest::getDefaultParameters() const {
	const PropertyMap::Ptr map = EngineTest::getDefaultParameters();
	map->add<double>("MaxSepDistance","Seperation distance used by the PQP contact detection strategy (in mm).",DEFAULT_MAXSEPDISTANCE);
	map->add("Enable Correction","Used for debugging).",true);
	//map->add("Height","Height of nut (in meters).",DEFAULT_HEIGHT);
	//map->add("Restitution","Coefficient of restitution.",DEFAULT_RESTITUTION);
	map->add<double>("ContactStrategyPQPUpdateThresholdAbsolute","In mm",DEFAULT_PQP_UPDATE_THRESHOLD);
	map->add<double>("ContactStrategyPQPUpdateThresholdLinear","",DEFAULT_PQP_UPDATE_THRESHOLD_LIN);
	map->add<double>("ContactStrategyPQPUpdateThresholdAngular","",DEFAULT_PQP_UPDATE_THRESHOLD_ANG);
	return map;
}
