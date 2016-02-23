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

#include "FixedWheel.hpp"
#include <cfloat>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

FixedWheel::FixedWheel(const std::string& name, const Transform3D<>& transform, double radius):
	RevoluteJoint(name,transform),
	_radius(radius)
{
}

FixedWheel::~FixedWheel() {
}

std::pair<Q, Q> FixedWheel::getBounds() const {
	std::pair<Q, Q> bounds;
	bounds.first = Q(1,-DBL_MAX);
	bounds.second = Q(1,DBL_MAX);
	return bounds;
}

void FixedWheel::setRadius(double radius) {
	_radius = radius;
}

double FixedWheel::getRadius() const {
	return _radius;
}

bool FixedWheel::isStearable() const {
	return false;
}
