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

WorkCell::Ptr ContactPQPBoxPlaneTest::getWC(const PropertyMap& map) {
	if (_wc.isNull()) {
		const WorkCell::Ptr wc = ownedPtr(new WorkCell("ContactPQPBoxPlaneTestWorkCell"));

		const static double RADIUS = 1.0;

		WorkCellBuilder builder;
		builder.addFloor(wc);
		builder.addBox(wc,RADIUS,RADIUS,RADIUS,-1);

		const StateStructure::Ptr stateStructure = wc->getStateStructure();
		State state = stateStructure->getDefaultState();
		wc->findFrame<MovableFrame>("Box")->setTransform(Transform3D<>(Vector3D<>::z()*0.4,EAA<>(-Pi/2.,0,0)),state);
		stateStructure->setDefaultState(state);

		_wc = wc;
	}
	return _wc;
}

PropertyMap::Ptr ContactPQPBoxPlaneTest::getDefaultParameters() const {
	const PropertyMap::Ptr map = ContactTest::getDefaultParameters();
	//map->add("Timestep","Timestep in milliseconds.",DEFAULT_DT*1000);
	return map;
}
