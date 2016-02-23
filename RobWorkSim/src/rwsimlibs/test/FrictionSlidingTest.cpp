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

#include "FrictionSlidingTest.hpp"
#include "DynamicWorkCellBuilder.hpp"

#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/sensor/BodyContactSensor.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsimlibs/ode/ODEContactStrategy.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::sensor;
using namespace rw::trajectory;
using namespace rwlibs::simulation;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::sensor;
using namespace rwsim::simulator;
using namespace rwsimlibs::test;

#define DEFAULT_TILT 0.4
#define DEFAULT_FRICTION 0.5

FrictionSlidingTest::FrictionSlidingTest() {
}

FrictionSlidingTest::~FrictionSlidingTest() {
}

static const double DT = 0.01;

bool FrictionSlidingTest::isEngineSupported(const std::string& engineID) const {
	if (engineID == "RWPhysics")
		return false;
	return true;
}

void FrictionSlidingTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
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

	const Body::Ptr control = dwc->findBody("Box");
	RW_ASSERT(!control.isNull());
	Transform3D<> target = control->getTransformW(state);
	target.P() -= Vector3D<>::z()*0.01;

	const SimulatedSensor::Ptr sensorFloor = dwc->findSensor("ContactSensorFloor");
	RW_ASSERT(!sensorFloor.isNull());
	const BodyContactSensor::Ptr contactSensorFloor = sensorFloor.cast<BodyContactSensor>();
	RW_ASSERT(!contactSensorFloor.isNull());

	DynamicSimulator::Ptr simulator = ownedPtr(new DynamicSimulator(dwc,engine));
	simulator->init(state);
	simulator->setTarget(control,target,state);

	State runState = simulator->getState();
	handle->append(TimedState(0,runState));

	Result fx("Forces x","The forces in x.");
	Result fy("Forces y","The forces in y.");
	Result fz("Forces z","The forces in z.");
	Result mu("Mu - Time","The friction coefficient (fx/fz).");
	Result muPos("Mu - Position","The friction coefficient as a function of position.");
	Result muVel("Mu - Velocity","The friction coefficient as a function of velocity.");
	fx.values.push_back(TimedQ(0,Q(1,0.)));
	fy.values.push_back(TimedQ(0,Q(1,0.)));
	fz.values.push_back(TimedQ(0,Q(1,0.)));
	mu.values.push_back(TimedQ(0,Q(1,0.)));
	muPos.values.push_back(TimedQ(0,Q(1,0.)));
	muVel.values.push_back(TimedQ(0,Q(1,0.)));

	static const double frequency = 0.5;
	static const double maxVel = 0.2;

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

		BOOST_FOREACH(const Contact3D& c, contactSensorFloor->getContacts(state)) {
			const double x = dot(c.f,Vector3D<>::x());
			const double y = dot(c.f,Vector3D<>::y());
			const double z = dot(c.f,Vector3D<>::z());
			fx.values.push_back(TimedQ(time,Q(1,x)));
			fy.values.push_back(TimedQ(time,Q(1,y)));
			fz.values.push_back(TimedQ(time,Q(1,z)));
			mu.values.push_back(TimedQ(time,Q(1,x/z)));
			muPos.values.push_back(TimedQ(c.p[0],Q(1,x/z)));
			muVel.values.push_back(TimedQ(control->getLinVelW(runState)[0],Q(1,x/z)));
		}

		if (time > 0.25) {
			const Vector3D<> dir = Vector3D<>::x();
			const double velocity = maxVel*std::sin(frequency*2*Pi*(time-0.25));
			simulator->setTarget(control,VelocityScrew6D<double>(velocity*dir,EAA<>(0,0,0)));
		}

		handle->append(TimedState(time,runState));
		handle->callback(time,failed,false);
	} while (time <= getRunTime() && !handle->isAborted());
	handle->append(fx);
	handle->append(fy);
	handle->append(fz);
	handle->append(mu);
	handle->append(muPos);
	handle->append(muVel);
	handle->callback(time,failed,true);

	std::stringstream errString;
	if (failed) {
		errString << "failed at time " << failTime << ": " << handle->getError();
		handle->setError(errString.str());
	}

	simulator->exitPhysics();
}

double FrictionSlidingTest::getRunTime() const {
	return 5;
}

rw::common::Ptr<DynamicWorkCell> FrictionSlidingTest::getDWC(const PropertyMap& map) {
	static const double radius = 0.025;
	static const double length = 0.1;
	static const double gap = 0.01;
	const double tilt = map.get("Tilt",DEFAULT_TILT);
	const double friction = map.get("Friction",DEFAULT_FRICTION);

	const WorkCell::Ptr wc = ownedPtr(new WorkCell("FrictionSlidingTestWorkCell"));
	const DynamicWorkCell::Ptr dwc = ownedPtr(new DynamicWorkCell(wc));

	DynamicWorkCellBuilder builder;
	builder.addFloor(dwc,"Floor");
	builder.addBox(dwc,2*radius,2*radius,radius,0);
	builder.addCylinder(dwc,radius,length,7850,"Cylinder","Box");
	builder.addMaterialData(dwc,friction,0);

	const Object::Ptr cylObject = wc->findObject("Cylinder");
	cylObject->getGeometry()[0]->setTransform(Transform3D<>(Vector3D<>::z()*(length/2.)));
	cylObject->getModels()[0]->setTransform(Transform3D<>(Vector3D<>::z()*(length/2.)));

	const StateStructure::Ptr stateStructure = wc->getStateStructure();
	State state = stateStructure->getDefaultState();
	const Rotation3D<> rot = EAA<>(0,-Pi/2-tilt,0).toRotation3D();
	const Vector3D<> P = Vector3D<>::z()*(gap+sin(tilt)*(length+gap+radius/2.)+cos(tilt)*radius)+Vector3D<>::x()*(cos(tilt)*(length+gap+radius/2.)-sin(tilt)*radius);
	wc->findFrame<MovableFrame>("Box")->setTransform(Transform3D<>(P,rot),state);
	wc->findFrame<MovableFrame>("Cylinder")->setTransform(Transform3D<>(Vector3D<>::z()*(radius/2.+gap)),state);
	stateStructure->setDefaultState(state);

	dwc->setGravity(Vector3D<>(0,0,-9.82));
	builder.contactsExclude(dwc,"Box","*");

	const Body::Ptr controlBody = dwc->findBody("Box");
	const RigidBody::Ptr pegBody = dwc->findBody("Cylinder").cast<RigidBody>();
	const Constraint::Ptr constraint = ownedPtr(new Constraint("SpringConstraint",Constraint::Free,controlBody.get(),pegBody.get()));
	Constraint::SpringParams spring;
	spring.enabled = true;
	const double Nsample = 8; // 8 timesteps in one period
	const double compFactor = Nsample*Nsample*DT*DT/(4*Pi*Pi);
	const double mass = pegBody->getMass();
	const InertiaMatrix<>& inertia = pegBody->getBodyInertia();
	const double inX = Vector3D<>::x().e().transpose()*inertia.e()*Vector3D<>::x().e();
	const double inY = Vector3D<>::y().e().transpose()*inertia.e()*Vector3D<>::y().e();
	const double inZ = Vector3D<>::z().e().transpose()*inertia.e()*Vector3D<>::z().e();
	Eigen::VectorXd massVec(6);
	massVec << mass, mass, mass, inX, inY, inZ;
	const Eigen::VectorXd compliance = compFactor*massVec.cwiseInverse();
	const double dampRatio = 1; // critical damping
	const Eigen::VectorXd damping = 2*dampRatio*(massVec.cwiseProduct(compliance.cwiseInverse())).cwiseSqrt();
	spring.compliance = compliance.asDiagonal();
	spring.damping = damping.asDiagonal();
	constraint->setSpringParams(spring);
	constraint->setTransform(Transform3D<>(Vector3D<>::z()*(radius/2.+gap)));
	dwc->addConstraint(constraint);

	// Finally add sensors
	const BodyContactSensor::Ptr sensorFloor = ownedPtr(new BodyContactSensor("ContactSensorFloor",wc->findFrame("Floor")));
	dwc->addSensor(sensorFloor);

	return dwc;
}

PropertyMap::Ptr FrictionSlidingTest::getDefaultParameters() const {
	const PropertyMap::Ptr map = EngineTest::getDefaultParameters();
	map->add("Tilt","Tilt in radians.",DEFAULT_TILT);
	map->add("Friction","Friction coefficient.",DEFAULT_FRICTION);
	return map;
}
