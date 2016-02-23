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

#include "IntegratorSpringTest.hpp"

#include <rw/geometry/Cylinder.hpp>
#include <rw/geometry/Sphere.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/RigidObject.hpp>
#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/dynamics/Constraint.hpp>
#include <rwsim/dynamics/FixedBody.hpp>
//#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>
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

static const double DT = 0.01;
#define RADIUS 0.1

IntegratorSpringTest::IntegratorSpringTest(const std::string& integratorType):
	_integratorType(integratorType)
{
}

IntegratorSpringTest::~IntegratorSpringTest() {
	if (!(_dwc == NULL)) {
		_dwc = NULL;
	}
}

bool IntegratorSpringTest::isEngineSupported(const std::string& engineID) const {
	if (engineID == "RWPhysics")
		return false;
	return true;
}

void IntegratorSpringTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
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
	}
	State state = dwc->getWorkcell()->getDefaultState();

	const Body::Ptr body = dwc->findBody("Object");
	RW_ASSERT(!body.isNull());
	const RigidBody::Ptr rbody = body.cast<RigidBody>();
	RW_ASSERT(!rbody.isNull());

	DynamicSimulator::Ptr simulator = ownedPtr(new DynamicSimulator(dwc,engine));
	simulator->init(state);

	State runState = simulator->getState();
	handle->append(TimedState(0,runState));

	Result result("Position in z","The z-coordinate of the object.");
	Result ref("Expected position","The analytical correct position.");
	Result deviation("Deviation","The deviation from the expected position.");
	Result energy("Energy","The total energy.");
	Result springEnergy("Spring Energy","The spring energy.");
	const Transform3D<> Tinit = rbody->getTransformW(state);
	const double k = 1000;
	//const double eqPos = - rbody->getMass()*9.82 / k+0.12;
	result.values.push_back(TimedQ(0,Q(1,Tinit.P()[2])));
	ref.values.push_back(TimedQ(0,Q(1,reference(0.0))));
	deviation.values.push_back(TimedQ(0,Q(1,Tinit.P()[2]-reference(0.0))));
	energy.values.push_back(TimedQ(0,Q(1,rbody->calcEnergy(state,Vector3D<>(0,0,-9.82),Vector3D<>::zero())+0.5*k*0.12*0.12)));
	springEnergy.values.push_back(TimedQ(0,Q(1,0.5*k*0.12*0.12)));

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
		const Transform3D<> T = rbody->getTransformW(runState);
		const double pos = T.P()[2];
		result.values.push_back(TimedQ(time,Q(1,pos)));
		ref.values.push_back(TimedQ(time,Q(1,reference(time))));
		deviation.values.push_back(TimedQ(time,Q(1,reference(time)-pos)));
		energy.values.push_back(TimedQ(time,Q(1,rbody->calcEnergy(runState,Vector3D<>(0,0,-9.82),Vector3D<>::zero())+0.5*k*(pos-0.12)*(pos-0.12))));
		springEnergy.values.push_back(TimedQ(time,Q(1,0.5*k*(pos-0.12)*(pos-0.12))));

		handle->append(TimedState(time,runState));
		handle->callback(time,failed,false);
	} while (time <= getRunTime() && !handle->isAborted());
	handle->append(result);
	handle->append(ref);
	handle->append(deviation);
	handle->append(energy);
	handle->append(springEnergy);
	handle->callback(time,failed,true);

	std::stringstream errString;
	if (failed) {
		errString << "failed at time " << failTime << ": " << handle->getError();
		handle->setError(errString.str());
	}

	simulator->exitPhysics();
}

double IntegratorSpringTest::getRunTime() const {
	return 5;
}

rw::common::Ptr<DynamicWorkCell> IntegratorSpringTest::getDWC(const PropertyMap& map) {
	if (_dwc == NULL)
		_dwc = makeDWC(_integratorType);
	return _dwc;
}

DynamicWorkCell::Ptr IntegratorSpringTest::makeDWC(const std::string& integratorType) {
	static double r = RADIUS;

	const Transform3D<> Tf = Transform3D<>(Vector3D<>::z()*r*1.2,RPY<>(0,0,0).toRotation3D());
	FixedFrame* const fframe = new FixedFrame("FixedObject",Tf);
	//MovableFrame* const fframe = new MovableFrame("FixedObject");
	MovableFrame* const mframe = new MovableFrame("Object");
	const GeometryData::Ptr cylinderData = ownedPtr(new Cylinder(r*3/4,r/5));
	const GeometryData::Ptr sphereData = ownedPtr(new Sphere(r));
	Geometry::Ptr cylinder = ownedPtr(new Geometry(cylinderData, "CylinderGeo"));
	Geometry::Ptr sphere = ownedPtr(new Geometry(sphereData, "SphereGeo"));
	//const Transform3D<> Tcyl(Vector3D<>(0,0,0));
	//const Transform3D<> Tsphere(Vector3D<>(0,0,0));
	//cylinder->setTransform(Tcyl);
	//sphere->setTransform(Tsphere);
	RigidObject::Ptr fobject = ownedPtr(new RigidObject(fframe));
	RigidObject::Ptr mobject = ownedPtr(new RigidObject(mframe));
	fobject->addGeometry(cylinder);
	mobject->addGeometry(sphere);

	// Visualization
	// Vector3D<float>(227.f,104.f,12.f)/255.f
	const Model3D::Material material("Material",227.f/255.f,104.f/255.f,12.f/255.f);
	// Vector3D<float>(64.f,116.f,142.f)/255.f
	const Model3D::Material materialB("Material",64.f/255.f,116.f/255.f,142.f/255.f);
	//const Model3D::Material material("Material",0.2f,0.2f,0.6f);
	const Model3D::Ptr cylinderModel = ownedPtr(new Model3D("CylinderGeo"));
	const Model3D::Ptr sphereModel = ownedPtr(new Model3D("SphereGeo"));
	cylinderModel->addTriMesh(materialB,*cylinderData->getTriMesh());
	sphereModel->addTriMesh(material,*sphereData->getTriMesh());
	//cylinderModel->setTransform(Tcyl);
	//sphereModel->setTransform(Tsphere);
	fobject->addModel(cylinderModel);
	mobject->addModel(sphereModel);

	const WorkCell::Ptr wc = ownedPtr(new WorkCell("IntegratorSpringTestWorkCell"));
	wc->addFrame(fframe,wc->getWorldFrame()); // takes ownership
	wc->addFrame(mframe,wc->getWorldFrame()); // takes ownership
	wc->add(fobject);
	wc->add(mobject);

	const StateStructure::Ptr stateStructure = wc->getStateStructure();
	State state = stateStructure->getDefaultState();
	mframe->setTransform(Transform3D<>(Vector3D<>::zero(),RPY<>(0,0,0).toRotation3D()),state);
	stateStructure->setDefaultState(state);

	BodyInfo info;
	info.material = "Plastic";
	info.objectType = "hardObj";
	info.mass = r*r*r*Pi*4/3*7850; // steel density: 7850 kg/m3
	std::cout << "Mass: " << info.mass << std::endl;
	//info.inertia = InertiaMatrix<>(0.0023164,0,0,0,0.0023164,0,0,0,0.0011243);
	const double in = info.mass*r*r*2/3;
	info.inertia = InertiaMatrix<>(in,in,in);
	if (integratorType == "")
		info.integratorType = "Heun";
	else
		info.integratorType = integratorType;
	//const Body::Ptr fbody = ownedPtr(new KinematicBody(info,fobject));
	const Body::Ptr fbody = ownedPtr(new FixedBody(info,fobject));
	const Body::Ptr rbody = ownedPtr(new RigidBody(info,mobject));

	const DynamicWorkCell::Ptr dwc = ownedPtr(new DynamicWorkCell(wc));
	dwc->setGravity(Vector3D<>(0,0,-9.82));
	dwc->getMaterialData().add("Plastic","");
	dwc->getContactData().add("hardObj","A hard object. with low elasticity");
	ContactDataMap::NewtonData ndata;
	ndata.cr = 0;
	dwc->getContactData().addNewtonData("hardObj","hardObj",ndata);
	dwc->addBody(fbody);
	dwc->addBody(rbody);

	const Constraint::Ptr constraint = ownedPtr(new Constraint("Spring",Constraint::Free,fbody.get(),rbody.get()));
	Constraint::SpringParams spring;
	spring.enabled = true;
	spring.compliance = Eigen::MatrixXd::Zero(6,6);
	spring.compliance(0,0) = 0.001;
	spring.compliance(1,1) = 0.001;
	spring.compliance(2,2) = 0.001;
	spring.compliance(3,3) = 1;
	spring.compliance(4,4) = 1;
	spring.compliance(5,5) = 1;
	spring.damping = Eigen::MatrixXd::Zero(6,6);
	constraint->setSpringParams(spring);
	dwc->addConstraint(constraint);

	return dwc;
}

double IntegratorSpringTest::reference(double t) const {
	const double k = 1000.;
	const double m = RADIUS*RADIUS*RADIUS*Pi*4/3*7850;
	const double f = -m*9.82;
	const double pinit = 0.12; // already 120 mm extended
	return (f - (f+k*pinit)*cos((sqrt(k)*t)/sqrt(m)))/k+pinit;
}
