/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "URDFModel.hpp"

#include <rw/math/RPY.hpp>

using namespace rw::loaders;
using namespace rw::math;

URDFModel::URDFModel() {
}

URDFModel::~URDFModel() {
}

void URDFModel::validate() const {
	// Construct map with links
	std::map<std::string, const Link*> strToFrame;
	BOOST_FOREACH(const Link &link, links) {
		strToFrame[link.name] = &link;
	}
	// Check that all parent links are valid links
	BOOST_FOREACH(const Joint &joint, joints) {
		if (strToFrame.find(joint.parent) == strToFrame.end()) {
			RW_THROW( "Joint " << joint.name << " refers to a frame \"" << joint.parent << "\" which has not been declared!");
		}
	}
	BOOST_FOREACH(const Sensor &sensor, sensors) {
		if (strToFrame.find(sensor.parent) == strToFrame.end()) {
			RW_THROW( "Sensor " << sensor.name << " refers to a frame \"" << sensor.parent << "\" which has not been declared!");
		}
	}
	// Check that all child links are valid links
	BOOST_FOREACH(const Joint &joint, joints) {
		if (strToFrame.find(joint.child) == strToFrame.end()) {
			RW_THROW( "Joint " << joint.name << " refers to a frame \"" << joint.child << "\" which has not been declared!");
		}
	}
	// Construct map with generic materials
	std::map<std::string, const Material*> strToMaterial;
	BOOST_FOREACH(const Material &material, materials) {
		strToMaterial[material.name] = &material;
	}
}

Transform3D<> URDFModel::Origin::toTransform() const {
	return Transform3D<>(xyz,RPY<>(rpy[2],rpy[1],rpy[0]));
}
