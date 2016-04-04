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

#include "PiHCompliantTest.hpp"
#include "DynamicWorkCellBuilder.hpp"

#include <rw/kinematics/FixedFrame.hpp>
#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/contacts/ContactStrategyCylinderTube.hpp>
#include <rwsim/contacts/ContactStrategyData.hpp>
#include <rwsim/contacts/ContactStrategyPQP.hpp>
#include <rwsim/contacts/ContactStrategyTracking.hpp>
#include <rwsim/contacts/TubePlaneStrategy.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/Constraint.hpp>
#include <rwsim/sensor/BodyContactSensor.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsimlibs/ode/ODEContactStrategy.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::proximity;
using namespace rw::sensor;
using namespace rw::trajectory;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::sensor;
using namespace rwsim::simulator;
using namespace rwsimlibs::test;

#define DEFAULT_DT 10.0 // ms
#define DEFAULT_MAXSEPDISTANCE 0.2 // mm
#define DEFAULT_TILT 0.1
#define DEFAULT_ALPHA 0.1
#define DEFAULT_ALPHA_THRESHOLD 0.01

PiHCompliantTest::PiHCompliantTest() {
}

PiHCompliantTest::~PiHCompliantTest() {
}

bool PiHCompliantTest::isEngineSupported(const std::string& engineID) const {
	if (engineID == "RWPhysics")
		return false;
	return true;
}

void PiHCompliantTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
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
	const double dt = parameters.get<double>("Timestep")/1000.;
	engine->setSimulatorLog(verbose);
	engine->load(dwc);
	const ContactDetector::Ptr detector = ownedPtr(new ContactDetector(dwc->getWorkcell()));
	if (engineID == "ODE") {
		const ContactStrategy::Ptr odeStrat = ownedPtr(new ODEContactStrategy());
		//const ContactStrategy::Ptr pqpStrat = ownedPtr(new ContactStrategyPQP());
		odeStrat->setPropertyMap(dwc->getEngineSettings());
		//pqpStrat->setPropertyMap(dwc->getEngineSettings());
		detector->addContactStrategy(odeStrat,0);
		//detector->addContactStrategy(pqpStrat,1);
	} else if (engineID == "TNT") {
		const ContactStrategy::Ptr pqpStrat = ownedPtr(new ContactStrategyPQP());
		pqpStrat->setPropertyMap(dwc->getEngineSettings());
		detector->addContactStrategy(pqpStrat);
	} else {
		detector->setDefaultStrategies(dwc->getEngineSettings());
	}
	if (engineID != "Bullet") {
		if (!engine->setContactDetector(detector)) {
			handle->setError("Engine did not accept a ContactDetector.");
			return;
		}
	}
	detector->printStrategyTable(std::cout);
	State state = dwc->getWorkcell()->getDefaultState();

	DynamicSimulator::Ptr simulator = ownedPtr(new DynamicSimulator(dwc,engine));

	const BodyContactSensor::Ptr sensorCyl = dwc->findSensor<BodyContactSensor>("ContactSensorCylinder");
	const BodyContactSensor::Ptr sensorFloor = dwc->findSensor<BodyContactSensor>("ContactSensorFloor");
	if (sensorCyl.isNull()) {
		handle->setError("Could not find BodyContactSensor with name \"ContactSensorCylinder\"");
		handle->callback(0,true,true);
		return;
	}
	if (sensorFloor.isNull()) {
		handle->setError("Could not find BodyContactSensor with name \"ContactSensorFloor\"");
		handle->callback(0,true,true);
		return;
	}

	try {
		simulator->init(state);
	} catch(Exception& e) {
		handle->setError("Simulator could not be initialized: " + e.getMessage().getFullText());
		handle->callback(0,true,true);
		return;
	}

	State runState = simulator->getState();
	handle->append(TimedState(0,runState));

	Result cylRes("Cylinder Forces","Forces detected by BodyContactSensor on cylinder.");
	Result floorRes("Floor Forces","Forces detected by BodyContactSensor on floor.");
	Result floorTotal("Floor Total Force","Summation of the floor forces.");
	Result contactsFloor("Contacts Floor","Number of contacts.");
	Result minDistFloor("Minimum Distance Floor","The minimum distance at floor.");
	Result minDistCyl("Minimum Distance Cylinder","The minimum distance at cylinder.");
	Result timing("Timing","The time for one step.");

	ContactStrategy::Ptr stratFloor = ownedPtr(new TubePlaneStrategy());
	ProximityModel::Ptr pmodelFloor = stratFloor->createModel();
	ProximityModel::Ptr pmodelTube = stratFloor->createModel();

	ContactStrategy::Ptr stratCylTube = ownedPtr(new ContactStrategyCylinderTube());
	ProximityModel::Ptr pmodelCylinder = stratCylTube->createModel();
	ProximityModel::Ptr pmodelTubeCyl = stratCylTube->createModel();

	BOOST_FOREACH(const Geometry::Ptr geo, dwc->findBody("Floor")->getGeometry(state)) {
		stratFloor->addGeometry(pmodelFloor.get(),geo);
	}
	BOOST_FOREACH(const Geometry::Ptr geo, dwc->findBody("Tube")->getGeometry(state)) {
		stratFloor->addGeometry(pmodelTube.get(),geo);
		stratCylTube->addGeometry(pmodelTubeCyl.get(),geo);
	}
	BOOST_FOREACH(const Geometry::Ptr geo, dwc->findBody("Cylinder")->getGeometry(state)) {
		stratCylTube->addGeometry(pmodelCylinder.get(),geo);
	}

	ContactStrategyTracking trackFloor;
	ContactStrategyTracking trackCyl;
	ContactStrategyData dataFloor;
	ContactStrategyData dataCyl;

	// Do a dummy check that causes contact tracking from beginning
	try {
		Transform3D<> wTtube = dwc->findBody("Tube").cast<RigidBody>()->getTransformW(state);
		Transform3D<> wTcyl = dwc->findBody("Cylinder").cast<RigidBody>()->getTransformW(state);
		wTtube.P() -= Vector3D<>::z()*0.001;
		wTcyl.P() -= Vector3D<>::z()*0.012;
		stratFloor->findContacts(pmodelTube, wTtube, pmodelFloor, Transform3D<>::identity(), dataFloor, trackFloor);
		wTtube.P() += Vector3D<>::z()*0.001;
		stratCylTube->findContacts(pmodelCylinder, wTcyl, pmodelTubeCyl, wTtube, dataCyl, trackCyl);
	} catch(const Exception& e) {
		handle->setError(e.what());
		handle->callback(0,true,true);
		return;
	}

	double time = 0;
	double failTime = -1;
	bool failed = false;
	bool controllerSet = false;
	do {
		long long start = TimerUtil::currentTimeMs();
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
		long long end = TimerUtil::currentTimeMs();
		timing.values.push_back(TimedQ(time,Q(1,end-start)));

		time = simulator->getTime();

		{
			const std::vector<Contact3D>& contacts = sensorCyl->getContacts(state);
			Q values(contacts.size());
			for (std::size_t i = 0; i < contacts.size(); i++) {
				values[i] = contacts[i].f.norm2();
			}
			cylRes.values.push_back(TimedQ(time,values));
		}
		{
			const std::vector<Contact3D>& contacts = sensorFloor->getContacts(state);
			Q values(contacts.size());
			Vector3D<> sum;
			for (std::size_t i = 0; i < contacts.size(); i++) {
				values[i] = contacts[i].f.norm2();
				sum += contacts[i].f;
			}
			floorRes.values.push_back(TimedQ(time,values));
			floorTotal.values.push_back(TimedQ(time,Q(1,sum.norm2())));
			contactsFloor.values.push_back(TimedQ(time,Q(1,contacts.size())));
		}

		const static Transform3D<> wTfloor = Transform3D<>::identity();
		const Transform3D<> wTtube = dwc->findBody("Tube").cast<RigidBody>()->getTransformW(runState);
		const Transform3D<> wTcyl = dwc->findBody("Cylinder").cast<RigidBody>()->getTransformW(runState);
		std::vector<Contact> contactsFloor;
		std::vector<Contact> contactsCyl;
		try {
			contactsFloor = stratFloor->findContacts(pmodelTube, wTtube, pmodelFloor, wTfloor, dataFloor, trackFloor);
			contactsCyl = stratCylTube->findContacts(pmodelCylinder, wTcyl, pmodelTubeCyl, wTtube, dataCyl, trackCyl);
		} catch(const Exception& e) {
			std::cout << "exception: " << e.what() << std::endl;
		}

		if (contactsFloor.size() > 0) {
			double minDist = -1;
			BOOST_FOREACH(const Contact& c, contactsFloor) {
				if(-c.getDepth() < minDist || minDist == -1) {
					minDist = -c.getDepth();
				}
			}
			minDistFloor.values.push_back(TimedQ(time,Q(1,minDist)));
		}
		if (contactsCyl.size() > 0) {
			double minDist = -1;
			BOOST_FOREACH(const Contact& c, contactsCyl) {
				if(-c.getDepth() < minDist || minDist == -1) {
					minDist = -c.getDepth();
				}
			}
			minDistCyl.values.push_back(TimedQ(time,Q(1,minDist)));
		}

		if (!controllerSet && time > 0.2) {
			const Body::Ptr box = dwc->findBody("Box");
			const Transform3D<> T = box->getTransformW(runState);
			simulator->setTarget(box,Transform3D<>(T.P()-Vector3D<>::z()*0.0135,T.R()));
			controllerSet = true;
		}

		handle->append(TimedState(time,runState));
		handle->callback(time,failed,false);
	} while (time <= getRunTime() && !handle->isAborted());
	handle->append(cylRes);
	handle->append(floorRes);
	handle->append(floorTotal);
	handle->append(contactsFloor);
	handle->append(minDistFloor);
	handle->append(minDistCyl);
	handle->append(timing);
	handle->callback(time,failed,true);

	std::stringstream errString;
	if (failed) {
		errString << "failed at time " << failTime << ": " << handle->getError();
		handle->setError(errString.str());
	}

	simulator->exitPhysics();
}

double PiHCompliantTest::getRunTime() const {
	return 2;
}

DynamicWorkCell::Ptr PiHCompliantTest::getDWC(const PropertyMap& map) {
	static const double radius = 0.025;
	static const double rSmall = radius-1e-3;
	static const double thickness = 0.005;
	static const double length = 0.1;
	static const double gap = 0.01;
	const double tilt = map.get("Tilt",DEFAULT_TILT);
	const bool trimesh = map.get("TriMesh",false);
	const double alpha = map.get("TNTSolverIterativeSVDAlpha",DEFAULT_ALPHA);
	const double alpha_threshold = map.get("TNTSolverIterativeSVDAlphaThreshold",DEFAULT_ALPHA_THRESHOLD);
	const double dt = map.get<double>("Timestep")/1000.;

	const WorkCell::Ptr wc = ownedPtr(new WorkCell("PiHCompliantTestWorkCell"));
	const DynamicWorkCell::Ptr dwc = ownedPtr(new DynamicWorkCell(wc));

	DynamicWorkCellBuilder builder;
	builder.addFloor(dwc,"Floor");
	builder.addBoxKin(dwc,2*radius,2*radius,radius,0);
	builder.addCylinder(dwc,rSmall,length,7850,"Cylinder","Box",trimesh);
	builder.addTube(dwc,radius,thickness,length,7850,"Tube");
	builder.addMaterialData(dwc,0.1,0);

	{
		FixedFrame* const frameF = new FixedFrame("Spring",Transform3D<>(Vector3D<>::z()*(radius/2)));
		FixedFrame* const frameM = new FixedFrame("SpringMoving",Transform3D<>(-Vector3D<>::z()*gap));
		wc->addFrame(frameF,wc->findFrame("Box"));
		wc->addFrame(frameM,wc->findFrame("Cylinder"));
	}

	const Object::Ptr cylObject = wc->findObject("Cylinder");
	cylObject->getGeometry()[0]->setTransform(Transform3D<>(Vector3D<>::z()*(length/2.)));
	cylObject->getModels()[0]->setTransform(Transform3D<>(Vector3D<>::z()*(length/2.)));

	const StateStructure::Ptr stateStructure = wc->getStateStructure();
	State state = stateStructure->getDefaultState();
	const Rotation3D<> rot = EAA<>(0,-Pi/2-tilt,0).toRotation3D();
	const Vector3D<> P = Vector3D<>::z()*(length+gap+sin(tilt)*(length+gap+radius/2.)+cos(tilt)*rSmall)+Vector3D<>::x()*(cos(tilt)*(length+gap+radius/2.)-sin(tilt)*rSmall);
	wc->findFrame<MovableFrame>("Box")->setTransform(Transform3D<>(P,rot),state);
	wc->findFrame<MovableFrame>("Cylinder")->setTransform(Transform3D<>(Vector3D<>::z()*(radius/2.+gap)),state);
	wc->findFrame<MovableFrame>("Tube")->setTransform(Transform3D<>(Vector3D<>::z()*(length/2.+0.5e-3)),state);
	stateStructure->setDefaultState(state);

	dwc->setGravity(Vector3D<>(0,0,-9.82));
	builder.contactsExclude(dwc,"Box","*");

	const Body::Ptr controlBody = dwc->findBody("Box");
	const RigidBody::Ptr pegBody = dwc->findBody("Cylinder").cast<RigidBody>();
	const Constraint::Ptr constraint = ownedPtr(new Constraint("SpringConstraint",Constraint::Free,controlBody.get(),pegBody.get()));
	Constraint::SpringParams spring;
	spring.enabled = true;
	const double Nsample = 8; // 8 timesteps in one period
	const double dtMax = 0.01;
	RW_ASSERT(dt <= dtMax);
	const double compFactor = Nsample*Nsample*dtMax*dtMax/(4*Pi*Pi);
	const double mass = pegBody->getMass();
	const InertiaMatrix<>& inertia = pegBody->getBodyInertia();
	const double inX = Vector3D<>::x().e().transpose()*inertia.e()*Vector3D<>::x().e();
	const double inY = Vector3D<>::y().e().transpose()*inertia.e()*Vector3D<>::y().e();
	const double inZ = Vector3D<>::z().e().transpose()*inertia.e()*Vector3D<>::z().e();
	Eigen::VectorXd massVec(6);
	massVec << mass, mass, mass, inX, inY, inZ;
	const Eigen::VectorXd compliance = compFactor*massVec.cwiseInverse();
	std::cout << "stiffness: " << compliance.cwiseInverse() << std::endl;
	const double dampRatio = 1; // critical damping
	const Eigen::VectorXd damping = 2*dampRatio*(massVec.cwiseProduct(compliance.cwiseInverse())).cwiseSqrt();
	spring.compliance = compliance.asDiagonal();
	spring.damping = damping.asDiagonal();
	constraint->setSpringParams(spring);
	constraint->setTransform(Transform3D<>(Vector3D<>::z()*(radius/2.+gap)));
	dwc->addConstraint(constraint);

	// Finally add sensors
	const BodyContactSensor::Ptr sensorCyl = ownedPtr(new BodyContactSensor("ContactSensorCylinder",wc->findFrame("Cylinder")));
	const BodyContactSensor::Ptr sensorFloor = ownedPtr(new BodyContactSensor("ContactSensorFloor",wc->findFrame("Floor")));
	dwc->addSensor(sensorCyl);
	dwc->addSensor(sensorFloor);

	dwc->getEngineSettings().set("TNTSolverIterativeSVDAlpha",alpha);
	dwc->getEngineSettings().set("TNTSolverIterativeSVDAlphaThreshold",alpha_threshold);
	dwc->getEngineSettings().set("TNTSolverIterativeSVDPrecision",1e-4);

	const double dist = map.get<double>("MaxSepDistance")/1000.;
	const double correction_layer = map.get<double>("TNTCorrectionContactLayer")/1000.;
	dwc->getEngineSettings().set("MaxSepDistance",dist);
	dwc->getEngineSettings().set("TNTCorrectionContactLayer",correction_layer);
	dwc->getEngineSettings().set("TNTSolverIterativeSVDPrecision",1e-6);

	return dwc;
}

PropertyMap::Ptr PiHCompliantTest::getDefaultParameters() const {
	const PropertyMap::Ptr map = EngineTest::getDefaultParameters();
	map->add<double>("Timestep","In ms",DEFAULT_DT);
	map->add<double>("MaxSepDistance","Seperation distance used by the PQP contact detection strategy (in mm).",DEFAULT_MAXSEPDISTANCE);
	map->add<double>("TNTCorrectionContactLayer","Target for correction (should be same or less than MaxSepDistance!) (in mm).",DEFAULT_MAXSEPDISTANCE);
	map->add("Tilt","Amount of tilting (in radians).",DEFAULT_TILT);
	map->add("TriMesh","Use a trimesh representation (for contact detection with PQP strategy).",false);
	map->add("TNTSolverIterativeSVDAlpha","Penalty for uneven distribution.",DEFAULT_ALPHA);
	map->add("TNTSolverIterativeSVDAlphaThreshold","Penalty for uneven distribution.",DEFAULT_ALPHA_THRESHOLD);
	return map;
}
