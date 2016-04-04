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

#include "KVMTest.hpp"

#include <rw/trajectory/Trajectory.hpp>
#include <rw/trajectory/TrajectoryFactory.hpp>
#include <rwlibs/assembly/AssemblyControlResponse.hpp>
#include <rwlibs/assembly/AssemblyControlStrategy.hpp>
#include <rwlibs/assembly/AssemblyParameterization.hpp>
#include <rwlibs/assembly/AssemblyResult.hpp>
#include <rwlibs/assembly/AssemblyState.hpp>
#include <rwlibs/assembly/AssemblyTask.hpp>
#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
//#include <rwsim/sensor/BodyContactSensor.hpp>
#include <rwsim/simulator/AssemblySimulator.hpp>
#include <rwsimlibs/ode/ODEContactStrategy.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::sensor;
using namespace rw::trajectory;
using namespace rwlibs::assembly;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::loaders;
//using namespace rwsim::sensor;
using namespace rwsim::simulator;
using namespace rwsimlibs::test;

#define DEFAULT_DT 0.01
//#define DEFAULT_HEIGHT 0
//#define DEFAULT_RESTITUTION 0
#define DEFAULT_MAXSEPDISTANCE 0.2
#define DEFAULT_PQP_UPDATE_THRESHOLD 1
#define DEFAULT_PQP_UPDATE_THRESHOLD_LIN 2.
#define DEFAULT_PQP_UPDATE_THRESHOLD_ANG 0.25

namespace {
class KVMParameterization: public AssemblyParameterization {
public:
	typedef rw::common::Ptr<KVMParameterization> Ptr;

	KVMParameterization(const PropertyMap::Ptr map):
    			holeRadius(map->get<double>("HoleRadius")),
				pegRadius(map->get<double>("PegRadius")),
				theta(map->get<double>("theta",0)),
				phi(map->get<double>("phi",0)),
				x(map->get<double>("x",0)),
				y(map->get<double>("y",0)),
				pertScale(map->get<double>("pertScale",0))
	{
	}

	KVMParameterization(double holeRadius, double pegRadius, double theta,double phi, double distX, double distY, double pertScale = 1):
    			holeRadius(holeRadius),
				pegRadius(pegRadius),
				theta(theta),
				phi(phi),
				x(distX),
				y(distY),
				pertScale(pertScale)
	{
	}

	virtual ~KVMParameterization() {
	}

	PropertyMap::Ptr toPropertyMap() const {
		PropertyMap::Ptr map = ownedPtr(new PropertyMap());
		map->set("HoleRadius",holeRadius);
		map->set("PegRadius",pegRadius);
		map->set("theta",theta);
		map->set("phi",phi);
		map->set("x",x);
		map->set("y",y);
		map->set("pertScale",pertScale);
		return map;
	}

	AssemblyParameterization::Ptr clone() const {
		return ownedPtr(new KVMParameterization(holeRadius,pegRadius,theta, phi, x, y));
	}

	void reset(PropertyMap::Ptr pmap) {
		_pmap = pmap;
		holeRadius 	= pmap->get<double>("HoleRadius");
		pegRadius 	= pmap->get<double>("PegRadius");
		theta		= pmap->get<double>("theta",0);
		phi			= pmap->get<double>("phi",0);
		x			= pmap->get<double>("x",0);
		y			= pmap->get<double>("y",0);
		pertScale	= pmap->get<double>("pertScale",0);
	}

	static KVMParameterization::Ptr makeDefault() {
		// 										holeRadius,	pegRadius,	theta,			phi,			distX,		distY,	pertScale
		return ownedPtr(new KVMParameterization(0.0095,		0.009,		17.7*Deg2Rad,	11.25*Deg2Rad,	-0.0009,	0.01	));
	}

public:
	double holeRadius;
	double pegRadius;
	double theta;
	double phi;
	double x;
	double y;
	double pertScale;
};

struct Command {
	Command(): type(MoveLinear), angle(0), allowedZoffset(0) {}

	typedef enum Type {
		MoveLinear,
		Circle,
		FemaleTmaleTest
	} Type;

	bool test(AssemblyState::Ptr state) const {
		if (type != FemaleTmaleTest)
			return true;
		const Transform3D<> femTmale = state->femaleTmale;
		const Vector3D<> pfemTmale = femTmale.P();
		if(pfemTmale[2] > allowedZoffset) {
			std::cout << pfemTmale[2] << " > " << allowedZoffset << std::endl;
			return false;
		}
		return true;
	}

	Type type;
	Transform3D<> T;
	Vector3D<> rotationPoint;
	double angle;
	double allowedZoffset;
};

static std::vector<Command> MakeKVMPath(KVMParameterization::Ptr param) {
	const double x = param->x;
	const double y = param->y;
	const double theta = param->theta;
	const double phi = param->phi;

	const double x0 = 0.02;
	const double pRad = 0.018/2;
	const double hRad = 0.019/2;
	const double distTContact = 0.02;

	const double yOffset = hRad -cos(theta)*pRad -(sin(theta)*pRad-x)*tan(theta);
	const Vector3D<> Offset(0,-yOffset,x);
	const RPY<> Rot(0.0,0.0,theta);
	const Transform3D<> T1(Offset,Rot);
	const Transform3D<> T0(T1.P() + Vector3D<>(0,distTContact*sin(phi),distTContact*cos(phi)),T1.R());
	const Vector3D<> rotationPoint = Vector3D<>(0,-hRad -y*sin(theta),y*cos(theta));

	const Transform3D<> T2(rw::math::Vector3D<>(0, -yOffset -y * sin(theta) ,-x0));
	std::vector<Command> cmd(5);
	cmd[0].T = T0;
	cmd[1].T = T1;
	cmd[2].type = Command::FemaleTmaleTest;
	cmd[2].allowedZoffset = pRad*sin(theta) + 0.007 + cos(theta)*0.05;
	cmd[3].type = Command::Circle;
	cmd[3].rotationPoint = rotationPoint;
	cmd[3].angle = theta;
	cmd[4].T = T2;

	return cmd;
}

class CMDQListStrategy: public AssemblyControlStrategy {
public:
	typedef rw::common::Ptr<CMDQListStrategy> Ptr;

	CMDQListStrategy(const std::vector<Command>& cmd, const Transform3D<>& femaleTfemTcp, const Transform3D<>& maleTmaleTcp, const Transform3D<>& worldTfemale):
		_cmd(cmd),
		_FemaleTfemTcp(femaleTfemTcp),
		_MaleTmaleTcp(maleTmaleTcp),
		_trajectoryStratTime(0),
		_worldTFemale(worldTfemale),
		_evaluationSuccessfull(false)
	{
	}

	ControlState::Ptr createState() const {
		return ownedPtr(new StateMachine());
	}

	AssemblyControlResponse::Ptr update(AssemblyParameterization::Ptr parameters, AssemblyState::Ptr real, AssemblyState::Ptr assumed, ControlState::Ptr controlState, State &state, FTSensor* ftSensor, double time) const {
		RW_ASSERT(!real.isNull());
		RW_ASSERT(!controlState.isNull());

		const AssemblyControlResponse::Ptr response = ownedPtr(new AssemblyControlResponse());
		_state = controlState.cast<StateMachine>();

		for(std::size_t i = 0; i < _listOfTimedCommands.size(); i++) {
			if(time >= _listOfTimedCommands[i].getTime()) {
				if(!_listOfTimedCommands[i].getValue().test(real) ) {
					response->done = true;
					response->success = false;
					RW_WARN("Strategy failed!");
					return response;
				} else {
					_listOfTimedCommands.erase(_listOfTimedCommands.begin() + i );
				}
			}
		}

		if(_state->trajectoryCreated) {
			if(time > _trajectoryStratTime + _worldtendTtrajectory->duration()) {
				response->done = true;
				return response;
			}
			assumed->femaleTmale = _worldtendTtrajectory->x(time - _trajectoryStratTime);
			return NULL;
		}

		_state->trajectoryCreated = true;
		_trajectoryStratTime = time;
		_femaleOff = assumed->femaleOffset;

		const Transform3D<> femalecfTmalecf =  inverse(_FemaleTfemTcp) * assumed->femaleOffset * assumed->femaleTmale * _MaleTmaleTcp;
		_worldtendTtrajectory = generateTrajectory(femalecfTmalecf, assumed);

		response->done = false;
		response->type =  AssemblyControlResponse::POSITION_TRAJECTORY;
		response->worldTendTrajectory = _worldtendTtrajectory;

		return response;
	}

	Transform3D<> getApproach(AssemblyParameterization::Ptr parameters) {
		const Transform3D<> femaletcpTmaletcp = _cmd[0].T;
		return _FemaleTfemTcp*femaletcpTmaletcp*inverse(_MaleTmaleTcp);
	}

	std::string getDescription() {
		return "CMDQListStrategy";
	}

	AssemblyParameterization::Ptr createParameterization(const PropertyMap::Ptr map) {
		return ownedPtr(new AssemblyParameterization(map));
	}

	bool getEvaluationSuccessFull() {
		return _evaluationSuccessfull;
	}

	std::string getID() {
		return "rwsimlibs.test.KVMTest:CMDQListStrategy";
	}

private:
	class StateMachine: public ControlState {
	public:
		typedef rw::common::Ptr<StateMachine> Ptr;

		bool trajectoryCreated;
		int state;
		unsigned int counter;

		StateMachine():
			trajectoryCreated(false),
			state(1),
			counter(1000)
		{}
	};

	bool evaluateSimulation(AssemblyState::Ptr real) {
		auto holeTcup = real->femaleTmale;
		auto cupTtube = real->maleflexT[0];
		auto holeTMaleTCP = holeTcup*cupTtube * _MaleTmaleTcp;
		auto trans = holeTMaleTCP.P();
		if(	trans[2] < -0.01
				&& _evaluationSuccessfull
		)
			return true;
		return false;
	}

	Transform3DTrajectory::Ptr generateTrajectory(Transform3D<> HoleTdevice, AssemblyState::Ptr assumed) const {
		Path<rw::math::Transform3D<> > T3Dpath;
		std::vector<double> times;
		HoleTdevice = _cmd[0].T;
		const Transform3D<> femaleTmale_frame = _FemaleTfemTcp*_cmd[0].T*inverse(_MaleTmaleTcp);
		const double dt = 0.04;
		double currentTrajectoryTime = 0;
		T3Dpath.push_back(_worldTFemale *femaleTmale_frame);
		times.push_back(dt);
		T3Dpath.push_back(_worldTFemale *femaleTmale_frame);
		assumed->femaleTmale = femaleTmale_frame;

		while(_state->state < (int)_cmd.size()) {
			switch(_cmd[_state->state].type) {
			case Command::MoveLinear:
			{
				const Command& command = _cmd[_state->state];
				Vector3D<> TubeTtargetp = command.T.P()- HoleTdevice.P();
				VelocityScrew6D<> vTubeTtarget(inverse(HoleTdevice)*command.T);
				if (TubeTtargetp.norm2() <= 0.002){
					HoleTdevice.P() = command.T.P();
					_state->state++;
				}
				else
				{
					HoleTdevice.P() += TubeTtargetp*(0.002 / TubeTtargetp.norm2());
				}
			}
			break;
			case Command::Circle:
			{
				const Command& command = _cmd[_state->state];
				const Vector3D<>& rotationPoint = command.rotationPoint;
				const double dAngle = 0.01;
				double angleTotarget = RPY<>(HoleTdevice.R())[2];
				if (angleTotarget <= 0.01) {
					HoleTdevice = HoleTdevice *
							Transform3D<>(rotationPoint-HoleTdevice.P())*
							Transform3D<>(RPY<>(0,0,-angleTotarget).toRotation3D())*
							Transform3D<>(HoleTdevice.P()-rotationPoint);
					_state->state++;
				}
				else
				{
					HoleTdevice = HoleTdevice *
							Transform3D<>(rotationPoint-HoleTdevice.P())*
							Transform3D<>(RPY<>(0,0,-dAngle).toRotation3D())*
							Transform3D<>(HoleTdevice.P()-rotationPoint);
				}
			}
			break;
			case Command::FemaleTmaleTest:
			{
				std::cout << "TSIZE: " << times.size() << std::endl;
				const Timed<Command> tcmd(0.04*times.size(),_cmd[_state->state]);
				_listOfTimedCommands.push_back(tcmd);
				_state->state++;
			}
			break;
			default:
				_state->state++;
				break;
			}
			const Transform3D<> femaleTmaleTarget = inverse(inverse(_FemaleTfemTcp)*assumed->femaleOffset) * HoleTdevice* inverse(_MaleTmaleTcp);
			assumed->femaleTmale = femaleTmaleTarget;

			T3Dpath.push_back(_worldTFemale * femaleTmaleTarget);
			times.push_back(dt);
			currentTrajectoryTime += dt;
		}
		T3Dpath.push_back(T3Dpath.back());
		times.push_back(dt);

		return TrajectoryFactory::makeLinearTrajectory(T3Dpath,times);
	}

	std::vector<Command> _cmd;
	Transform3D<> _FemaleTfemTcp,_MaleTmaleTcp;
	mutable Transform3D<> _femaleOff;
	mutable double _trajectoryStratTime;
	mutable Transform3DTrajectory::Ptr _worldtendTtrajectory;
	mutable Transform3D<> _worldTFemale;
	mutable bool _evaluationSuccessfull;
	mutable rw::common::Ptr<StateMachine> _state;
	mutable std::vector<Timed<Command> > _listOfTimedCommands;
};
}

KVMTest::KVMTest() {
}

KVMTest::~KVMTest() {
}

bool KVMTest::isEngineSupported(const std::string& engineID) const {
	if (engineID == "RWPhysics")
		return false;
	return true;
}

void KVMTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
	const DynamicWorkCell::Ptr dwc = getDWC(parameters);
	if (dwc.isNull()) {
		handle->setError("Could not make engine.");
		return;
	}
	const double dt = parameters.get<double>("Timestep");

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

	const Transform3D<> femaleTfemTcp = Transform3D<>(Vector3D<>(0,0,0.0075));
	const Transform3D<> maleTmaleTcp = Transform3D<>(Vector3D<>(0,0,-0.05));

	PropertyMap paramMap = parameters.get<PropertyMap>("KVMParameterization");
	const KVMParameterization::Ptr param = ownedPtr(new KVMParameterization(&paramMap));
	// x, theta and phi is sampled
	const AssemblyTask::Ptr task = ownedPtr(new AssemblyTask());
	task->maleID = "Tube";
	task->femaleID = "Nut2";
	task->taskID = "1";
	task->generator = "KVMTest in rwsimlibs::test suite.";
	task->malePoseController = "Tube";
	std::vector<std::string> flexMale = {"Tube"};
	task->maleFlexFrames = flexMale;
	std::vector<std::string> flexFemale = {"Nut2"};
	task->femaleFlexFrames = flexFemale;
	task->maleFTSensor = "FTSensorMale";
	//task->bodyContactSensors.push_back("ContactSensorFeeder");
	//task->bodyContactSensors.push_back("ContactSensorTube");
	task->parameters = param;

	const Frame* const ACPframe = _dwc->getWorkcell()->findFrame("NutACP");
	if (ACPframe == NULL) {
		handle->setError("Could not find ACPframe.");
		return;
	}

	const Transform3D<> worldTfemale = ACPframe->getTransform(_dwc->getWorkcell()->getDefaultState());
	CMDQListStrategy::Ptr Qliststrat = ownedPtr(new CMDQListStrategy(MakeKVMPath(param),femaleTfemTcp,maleTmaleTcp,worldTfemale));
	task->strategy = Qliststrat;

	const std::vector<AssemblyTask::Ptr> tasks(1,task);

	AssemblySimulator::Ptr sim = NULL;
	if (engineID == "ODE") {
		const ContactDetector::Ptr detector = ownedPtr(new ContactDetector(dwc->getWorkcell()));
		const ContactStrategy::Ptr odeStrat = ownedPtr(new ODEContactStrategy());
		odeStrat->setPropertyMap(dwc->getEngineSettings());
		detector->addContactStrategy(odeStrat);
		sim = ownedPtr(new AssemblySimulator(_dwc,engineID,detector,verbose));
	} else {
		sim = ownedPtr(new AssemblySimulator(_dwc,engineID,NULL,verbose));
	}
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
		handle->callback(0,false,true);
		return;
	}
	const std::vector<AssemblyResult::Ptr> results = sim->getResults();

	if (results.size() > 0) {
		RW_ASSERT(results.size() == 1);
		//Result feederRes("Feeder Forces","Forces detected by BodyContactSensor on nut-feeder.");
		Result tubeRes("Tube Forces","Forces detected by BodyContactSensor on tube.");
		//Result feederTotal("Feeder Total Force","Summation of the feeder forces.");
		//Result contactsFeeder("Contacts Feeder","Number of contacts on feeder.");

		AssemblyTask::saveRWTask(tasks,"kvm_task_"+engineID+".assembly.xml");
		AssemblyResult::saveRWResult(results,"kvm_result_"+engineID+".assembly.xml");


		BOOST_FOREACH(const Timed<AssemblyState>& tstate, results[0]->realState) {
			const double time = tstate.getTime();
			const AssemblyState& state = tstate.getValue();
			const Wrench6D<> ft = state.ftSensorMale;
			tubeRes.values.push_back(TimedQ(time,Q(1,ft.force().norm2())));
		}
		handle->append(tubeRes);

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

double KVMTest::getRunTime() const {
	return 2.;
}

rw::common::Ptr<DynamicWorkCell> KVMTest::getDWC(const PropertyMap& map) {
	//const double restitution = map.get("Restitution",DEFAULT_RESTITUTION);
	//const double height = map.get("Height",DEFAULT_HEIGHT);
	if (_dwc.isNull()) {
		_dwc = DynamicWorkCellLoader::load("/home/thomas/Eclipse/RobWork/RobWorkSim/test/testfiles/scene/KVM/pih_kin_peg.dwc.xml");
		_defaultSettings = _dwc->getEngineSettings();
	}
	PropertyMap& settings = _dwc->getEngineSettings();
	settings = _defaultSettings;
	const double dist = map.get<double>("MaxSepDistance")/1000.;
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
	settings.set<int>("TNTCorrection",correction?1:0);
	return _dwc;
}

PropertyMap::Ptr KVMTest::getDefaultParameters() const {
	const PropertyMap::Ptr map = EngineTest::getDefaultParameters();
	map->add<double>("MaxSepDistance","Seperation distance used by the PQP contact detection strategy (in mm).",DEFAULT_MAXSEPDISTANCE);
	map->add<double>("TNTCorrectionContactLayer","Target for correction (should be same or less than MaxSepDistance!) (in mm).",DEFAULT_MAXSEPDISTANCE);
	map->add("Enable Correction","Used for debugging).",true);
	//map->add("Height","Height of nut (in meters).",DEFAULT_HEIGHT);
	//map->add("Restitution","Coefficient of restitution.",DEFAULT_RESTITUTION);
	map->add<double>("Timestep","",DEFAULT_DT);
	map->add<double>("ContactStrategyPQPUpdateThresholdAbsolute","In mm",DEFAULT_PQP_UPDATE_THRESHOLD);
	map->add<double>("ContactStrategyPQPUpdateThresholdLinear","",DEFAULT_PQP_UPDATE_THRESHOLD_LIN);
	map->add<double>("ContactStrategyPQPUpdateThresholdAngular","",DEFAULT_PQP_UPDATE_THRESHOLD_ANG);
	map->add("KVMParameterization","",*KVMParameterization::makeDefault()->toPropertyMap());
	return map;
}
