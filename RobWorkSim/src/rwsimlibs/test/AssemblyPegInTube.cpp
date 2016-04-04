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

#include "AssemblyPegInTube.hpp"

#include <rw/geometry/Cylinder.hpp>
#include <rw/geometry/Tube.hpp>
#include <rwlibs/assembly/AssemblyResult.hpp>
#include <rwlibs/assembly/AssemblyState.hpp>
#include <rwlibs/assembly/AssemblyTask.hpp>
#include <rwlibs/assembly/CircularPiHControlStrategy.hpp>
#include <rwlibs/assembly/CircularPiHParameterization.hpp>
#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsim/simulator/AssemblySimulator.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::assembly;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::loaders;
using namespace rwsim::simulator;
using namespace rwsimlibs::test;

#define DEFAULT_DT 0.01
#define DEFAULT_ANGLE 1.2
#define DEFAULT_DISTA 0
#define DEFAULT_DISTB 0

AssemblyPegInTube::AssemblyPegInTube() {
}

AssemblyPegInTube::~AssemblyPegInTube() {
}

bool AssemblyPegInTube::isEngineSupported(const std::string& engineID) const {
	if (engineID == "RWPhysics")
		return false;
	return true;
}

void AssemblyPegInTube::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
	const DynamicWorkCell::Ptr dwc = getDWC(parameters);
	if (dwc.isNull()) {
		handle->setError("Could not make engine.");
		return;
	}
	const double dt = parameters.get<double>("Timestep")/1000.;

	/*const BodyContactSensor::Ptr sensorFeeder = dwc->findSensor<BodyContactSensor>("ContactSensorFeeder");
	const BodyContactSensor::Ptr sensorTube = dwc->findSensor<BodyContactSensor>("ContactSensorTube");
	if (sensorFeeder.isNull()) {
		handle->setError("Could not find BodyContactSensor with name \"ContactSensorFeeder\"");
		handle->callback(0,true,true);
		return;
	}
	if (sensorTube.isNull()) {
		handle->setError("Could not find BodyContactSensor with name \"ContactSensorTube\"");
		handle->callback(0,true,true);
		return;
	}*/

	//const Transform3D<> femaleTfemTcp = Transform3D<>(Vector3D<>(0,0,0.0075));
	//const Transform3D<> maleTmaleTcp = Transform3D<>(Vector3D<>(0,0,-0.05));

	const double angle = parameters.get<double>("Angle");
	const double distA = parameters.get<double>("DistanceA")/1000.;
	const double distB = parameters.get<double>("DistanceB")/1000.;
	const rw::common::Ptr<const Tube> tubeGeo = _dwc->getWorkcell()->findObject("Tube")->getGeometry()[0]->getGeometryData().cast<Tube>();
	const rw::common::Ptr<const Cylinder> cylGeo = _dwc->getWorkcell()->findObject("Cylinder")->getGeometry()[0]->getGeometryData().cast<Cylinder>();
	const CircularPiHParameterization::Ptr param = ownedPtr( new CircularPiHParameterization(tubeGeo->getInnerRadius(),cylGeo->getRadius(),cylGeo->getHeight(),angle,distA,distB));

	// x, theta and phi is sampled
	const AssemblyTask::Ptr task = ownedPtr(new AssemblyTask());
	task->maleID = "Cylinder";
	task->femaleID = "Tube";
	task->taskID = "1";
	task->maleTCP = "Peg";
	task->femaleTCP = "Hole";
	task->generator = "AssemblyPegInTube in rwsimlibs::test suite.";
	task->malePoseController = "Box";
	std::vector<std::string> flexMale = {"Cylinder","Box"};
	task->maleFlexFrames = flexMale;
	std::vector<std::string> flexFemale = {"Tube"};
	task->femaleFlexFrames = flexFemale;
	task->maleFTSensor = "FTSensor";
	//task->bodyContactSensors.push_back("ContactSensorFeeder");
	//task->bodyContactSensors.push_back("ContactSensorTube");
	task->parameters = param;
/*
	const Frame* const ACPframe = _dwc->getWorkcell()->findFrame("NutACP");
	if (ACPframe == NULL) {
		handle->setError("Could not find ACPframe.");
		return;
	}

	const Transform3D<> worldTfemale = ACPframe->getTransform(_dwc->getWorkcell()->getDefaultState());
	*/
	CircularPiHControlStrategy::Ptr strat = ownedPtr(new CircularPiHControlStrategy());
	task->strategy = strat;

	const std::vector<AssemblyTask::Ptr> tasks(1,task);

	AssemblySimulator::Ptr sim = NULL;
	/*if (engineID == "ODE") {
		const ContactDetector::Ptr detector = ownedPtr(new ContactDetector(dwc->getWorkcell()));
		const ContactStrategy::Ptr odeStrat = ownedPtr(new ODEContactStrategy());
		odeStrat->setPropertyMap(dwc->getEngineSettings());
		detector->addContactStrategy(odeStrat);
		sim = ownedPtr(new AssemblySimulator(_dwc,engineID,detector,verbose));
	} else {*/
		sim = ownedPtr(new AssemblySimulator(_dwc,engineID,NULL,verbose));
	//}
	sim->setMaxSimTime(getRunTime());
	sim->setDt(dt);
	sim->setStoreExecutionData(true);

	//if(engineID == "TNTIsland")
		sim->setStartInApproach(true);

	sim->setTasks(tasks);
	sim->setStoreExecutionData(true);
	try {
		sim->start();
	} catch(const Exception& e) {
		handle->setError("Exception: " + e.getMessage().getFullText());
		handle->callback(0,true,true);
		return;
	}
	const std::vector<AssemblyResult::Ptr> results = sim->getResults();

	if (results.size() > 0) {
		RW_ASSERT(results.size() == 1);
		//Result feederRes("Feeder Forces","Forces detected by BodyContactSensor on nut-feeder.");
		//Result tubeRes("Tube Forces","Forces detected by BodyContactSensor on tube.");
		//Result feederTotal("Feeder Total Force","Summation of the feeder forces.");
		//Result contactsFeeder("Contacts Feeder","Number of contacts on feeder.");

		AssemblyTask::saveRWTask(tasks,"pit_task_"+engineID+".assembly.xml");
		AssemblyResult::saveRWResult(results,"pit_result_"+engineID+".assembly.xml");

		/*
		BOOST_FOREACH(const Timed<AssemblyState>& tstate, results[0]->realState) {
			const double time = tstate.getTime();
			const AssemblyState& state = tstate.getValue();
			const Wrench6D<> ft = state.ftSensorMale;
			tubeRes.values.push_back(TimedQ(time,Q(1,ft.force().norm2())));
		}
		handle->append(tubeRes);
		*/

		const double lastTime = results.back()->realState.back().getTime();
		if (results.back()->error != AssemblyResult::NONE) {
			handle->setError(results.back()->errorMessage);
			handle->callback(lastTime,false,true);
		} else {
			handle->callback(lastTime,true,true);
		}
	} else {
		handle->setError("No results generated in assembly simulation!");
		handle->callback(0,false,true);
	}
}

double AssemblyPegInTube::getRunTime() const {
	return 5.;
}

rw::common::Ptr<DynamicWorkCell> AssemblyPegInTube::getDWC(const PropertyMap& map) {
	//const double restitution = map.get("Restitution",DEFAULT_RESTITUTION);
	//const double height = map.get("Height",DEFAULT_HEIGHT);
	if (_dwc.isNull()) {
		_dwc = DynamicWorkCellLoader::load("/home/thomas/Eclipse/RobWork/RobWorkSim/test/testfiles/scenes/cylinder_tube_insertion/scene.dwc.xml");
		_defaultSettings = _dwc->getEngineSettings();
	}
	PropertyMap& settings = _dwc->getEngineSettings();
	settings = _defaultSettings;
	//double holeRadius, double holeLength, double pegRadius, double pegLength, double angle = 0, double distA = 0, double distB = 0
	/*const double dist = map.get<double>("MaxSepDistance")/1000.;
	const double correction_layer = map.get<double>("TNTCorrectionContactLayer")/1000.;
	const double pqp_threshold = map.get<double>("ContactStrategyPQPUpdateThresholdAbsolute")/1000.;
	const double pqp_threshold_lin = map.get<double>("ContactStrategyPQPUpdateThresholdLinear");
	const double pqp_threshold_ang = map.get<double>("ContactStrategyPQPUpdateThresholdAngular");
	const bool correction = map.get("Enable Correction",true);
	settings.set<double>("MaxSepDistance",dist);
	settings.set<double>("TNTCorrectionContactLayer",correction_layer);
	settings.set<double>("ContactStrategyPQPUpdateThresholdAbsolute",pqp_threshold);
	settings.set<double>("ContactStrategyPQPUpdateThresholdLinear",pqp_threshold_lin);
	settings.set<double>("ContactStrategyPQPUpdateThresholdAngular",pqp_threshold_ang);
	settings.set<int>("TNTCorrection",correction?1:0);*/
	return _dwc;
}

PropertyMap::Ptr AssemblyPegInTube::getDefaultParameters() const {
	const PropertyMap::Ptr map = EngineTest::getDefaultParameters();
	map->add<double>("Timestep","In ms",DEFAULT_DT*1000);
	map->add<double>("Angle","In radians",DEFAULT_ANGLE);
	map->add<double>("DistanceA","In mm",DEFAULT_DISTA);
	map->add<double>("DistanceB","In mm",DEFAULT_DISTB);
	/*map->add<double>("MaxSepDistance","Seperation distance used by the PQP contact detection strategy (in mm).",DEFAULT_MAXSEPDISTANCE);
	map->add<double>("TNTCorrectionContactLayer","Target for correction (should be same or less than MaxSepDistance!) (in mm).",DEFAULT_MAXSEPDISTANCE);
	map->add("Enable Correction","Used for debugging).",true);
	//map->add("Height","Height of nut (in meters).",DEFAULT_HEIGHT);
	//map->add("Restitution","Coefficient of restitution.",DEFAULT_RESTITUTION);
	map->add<double>("ContactStrategyPQPUpdateThresholdAbsolute","In mm",DEFAULT_PQP_UPDATE_THRESHOLD);
	map->add<double>("ContactStrategyPQPUpdateThresholdLinear","",DEFAULT_PQP_UPDATE_THRESHOLD_LIN);
	map->add<double>("ContactStrategyPQPUpdateThresholdAngular","",DEFAULT_PQP_UPDATE_THRESHOLD_ANG);
	*/
	return map;
}
