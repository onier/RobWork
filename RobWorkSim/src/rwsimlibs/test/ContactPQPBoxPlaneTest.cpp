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

#include "ContactPQPBoxPlaneTest.hpp"
#include "WorkCellBuilder.hpp"

#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/WorkCell.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rwsimlibs::test;

ContactPQPBoxPlaneTest::ContactPQPBoxPlaneTest() {
}

ContactPQPBoxPlaneTest::~ContactPQPBoxPlaneTest() {
}

#define LAYER 1e-5
WorkCell::Ptr ContactPQPBoxPlaneTest::getWC(const PropertyMap& map) {
	const bool trimesh = map.get<bool>("Trimesh");
	const double size = map.get<double>("Size");
	const WorkCell::Ptr wc = ownedPtr(new WorkCell("ContactPQPBoxPlaneTestWorkCell"));

	WorkCellBuilder builder;
	builder.addFloor(wc);
	builder.addBox(wc,size,size,size,-1,"Box",trimesh);

	const StateStructure::Ptr stateStructure = wc->getStateStructure();
	State state = stateStructure->getDefaultState();
	wc->findFrame<MovableFrame>("Box")->setTransform(Transform3D<>(Vector3D<>::z()*(size/2.+LAYER/2.),EAA<>(-Pi/2.,0,0)),state);
	stateStructure->setDefaultState(state);

	return wc;
}

std::map<std::string, State> ContactPQPBoxPlaneTest::getPoses(const PropertyMap& map) {
	const double size = map.get<double>("Size");
	std::map<std::string, State> poses;
	const WorkCell::Ptr wc = getWC(map);
	const State defState = wc->getDefaultState();
	State state;
	state = defState;
	wc->findFrame<MovableFrame>("Box")->setTransform(Transform3D<>(Vector3D<>::z()*0.4*size,EAA<>(-Pi/2.,0,0)),state);
	poses["Penetration"] = state;
	wc->findFrame<MovableFrame>("Box")->setTransform(Transform3D<>(Vector3D<>::z()*(size/2.+LAYER/2.),EAA<>(-Pi/2.,0,0)),state);
	poses["In Layer"] = state;
	wc->findFrame<MovableFrame>("Box")->setTransform(Transform3D<>(Vector3D<>::z()*(size/2.+LAYER/2.),EAA<>(-Pi/2.,0,0)),state);
	poses["In Track Layer"] = state;
	wc->findFrame<MovableFrame>("Box")->setTransform(Transform3D<>(Vector3D<>::z()*0.6*size,EAA<>(-Pi/2.,0,0)),state);
	poses["Increased Height"] = state;
	return poses;
}

PropertyMap::Ptr ContactPQPBoxPlaneTest::getDefaultParameters() const {
	const PropertyMap::Ptr map = ContactTest::getDefaultParameters();
	map->add("Size","Side length of the box (in meters).",0.1);
	map->add("Trimesh","Use trimesh representation for box.",true);
	return map;
}
