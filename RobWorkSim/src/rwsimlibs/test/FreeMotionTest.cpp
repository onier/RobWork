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

#include "FreeMotionTest.hpp"
#include "DynamicWorkCellBuilder.hpp"

#include <rw/geometry/Cylinder.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/RigidObject.hpp>
#include <rwsim/contacts/ContactDetector.hpp>
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

#define DEFAULT_DT 10 // in ms

FreeMotionTest::FreeMotionTest(const std::string& integratorType):
	_integratorType(integratorType)
{
}

FreeMotionTest::~FreeMotionTest() {
	if (!(_dwc == NULL)) {
		_dwc = NULL;
	}
}

bool FreeMotionTest::isEngineSupported(const std::string& engineID) const {
	if (engineID == "RWPhysics")
		return false;
	return true;
}

void FreeMotionTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
	const double dt = parameters.get<double>("Timestep")/1000.;

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
	Result reference("Expected position","The analytical correct position.");
	Result deviation("Deviation","The deviation from the expected position.");
	Result energy("Energy","The energy of the object.");
	const Transform3D<> Tinit = rbody->getTransformW(state);
	result.values.push_back(TimedQ(0,Q(1,Tinit.P()[2])));
	reference.values.push_back(TimedQ(0,Q(1,Tinit.P()[2])));
	deviation.values.push_back(TimedQ(0,Q(1,0.0)));
	energy.values.push_back(TimedQ(0,Q(1,rbody->calcEnergy(state,dwc->getGravity()))));

	double time = 0;
	double failTime = -1;
	bool failed = false;
	do {
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
		time = simulator->getTime();
		const Transform3D<> T = rbody->getTransformW(runState);
		result.values.push_back(TimedQ(time,Q(1,T.P()[2])));
		reference.values.push_back(TimedQ(time,Q(1,Tinit.P()[2]-0.5*9.82*time*time)));
		deviation.values.push_back(TimedQ(time,Q(1,Tinit.P()[2]-0.5*9.82*time*time-T.P()[2])));
		energy.values.push_back(TimedQ(time,Q(1,rbody->calcEnergy(runState,dwc->getGravity()))));

		handle->append(TimedState(time,runState));
		handle->callback(time,failed,false);
	} while (time <= getRunTime() && !handle->isAborted());
	handle->append(result);
	handle->append(reference);
	handle->append(deviation);
	handle->append(energy);
	handle->callback(time,failed,true);

	std::stringstream errString;
	if (failed) {
		errString << "failed at time " << failTime << ": " << handle->getError();
		handle->setError(errString.str());
	}

	simulator->exitPhysics();
}

double FreeMotionTest::getRunTime() const {
	return 1.;
}

rw::common::Ptr<DynamicWorkCell> FreeMotionTest::getDWC(const PropertyMap& map) {
	if (_dwc == NULL) {
		_dwc = makeDWC(_integratorType);
		_dwc->setGravity(Vector3D<>(0,0,-9.82));
	}
	return _dwc;
}

PropertyMap::Ptr FreeMotionTest::getDefaultParameters() const {
	const PropertyMap::Ptr map = EngineTest::getDefaultParameters();
	map->add<double>("Timestep","Default timestep size in ms.",DEFAULT_DT);
	return map;
}

DynamicWorkCell::Ptr FreeMotionTest::makeDWC(const std::string& integratorType) {
	MovableFrame* const frame = new MovableFrame("Object");
	const GeometryData::Ptr cylinderData = ownedPtr(new Cylinder(0.015,0.1));
	const GeometryData::Ptr cylinderEndData = ownedPtr(new Cylinder(0.045,0.02));
	Geometry::Ptr cylinder = ownedPtr(new Geometry(cylinderData, "CylinderGeo"));
	Geometry::Ptr cylinderEnd = ownedPtr(new Geometry(cylinderEndData, "CylinderEndGeo"));
	const Transform3D<> Tcyl(Vector3D<>(0,0,-27./700.));
	const Transform3D<> TcylEnd(Vector3D<>(0,0,0.06-27./700.));
	cylinder->setTransform(Tcyl);
	cylinderEnd->setTransform(TcylEnd);
	RigidObject::Ptr object = ownedPtr(new RigidObject(frame));
	object->addGeometry(cylinder);
	object->addGeometry(cylinderEnd);

	// Visualization
	// Vector3D<float>(227.f,104.f,12.f)/255.f
	const Model3D::Material material("Material",227.f/255.f,104.f/255.f,12.f/255.f);
	const Model3D::Ptr cylinderModel = ownedPtr(new Model3D("CylinderGeo"));
	const Model3D::Ptr cylinderEndModel = ownedPtr(new Model3D("CylinderEndGeo"));
	cylinderModel->addTriMesh(material,*cylinderData->getTriMesh());
	cylinderEndModel->addTriMesh(material,*cylinderEndData->getTriMesh());
	cylinderModel->setTransform(Tcyl);
	cylinderEndModel->setTransform(TcylEnd);
	object->addModel(cylinderModel);
	object->addModel(cylinderEndModel);

	const WorkCell::Ptr wc = ownedPtr(new WorkCell("FreeMotionTestWorkCell"));
	wc->addFrame(frame,wc->getWorldFrame()); // takes ownership of the MavableFrame
	wc->add(object);

	const StateStructure::Ptr stateStructure = wc->getStateStructure();
	State state = stateStructure->getDefaultState();
	frame->setTransform(Transform3D<>(Vector3D<>::zero(),RPY<>(0,Pi/4,0).toRotation3D()),state);
	stateStructure->setDefaultState(state);

	BodyInfo info;
	info.material = "Plastic";
	info.objectType = "hardObj";
	info.mass = 1.554; // steel density: 7850 kg/m3
	info.inertia = InertiaMatrix<>(0.0023164,0,0,0,0.0023164,0,0,0,0.0011243);
	if (integratorType == "")
		info.integratorType = "Heun";
	else
		info.integratorType = integratorType;
	RigidBody::Ptr rbody = ownedPtr(new RigidBody(info,object));

	const DynamicWorkCell::Ptr dwc = ownedPtr(new DynamicWorkCell(wc));
	dwc->getMaterialData().add("Plastic","");
	dwc->getContactData().add("hardObj","A hard object. with low elasticity");
	ContactDataMap::NewtonData ndata;
	ndata.cr = 0;
	dwc->getContactData().addNewtonData("hardObj","hardObj",ndata);
	dwc->addBody(rbody);
	return dwc;
}
