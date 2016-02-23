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

#include "SteeredWheel.hpp"
#include <cfloat>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

SteeredWheel::SteeredWheel(const std::string& name, const Transform3D<>& transform, const std::string &wheel, double offset):
	RevoluteJoint(name,transform),
	_wheelName(wheel),
	_wheel(NULL),
	_radius(0),
	_offset(offset),
	_transform(transform)
{
}

SteeredWheel::~SteeredWheel() {
}

void SteeredWheel::setBounds(const std::pair<const math::Q, const math::Q>& bounds) {
	std::pair<Q, Q> bound = std::make_pair<Q, Q>(Q(1,bounds.first[0]),Q(1,bounds.second[0]));
	Joint::setBounds(bound);
}

std::pair<Q, Q> SteeredWheel::getBounds() const {
	std::pair<Q, Q> bounds;
	bounds.first = Q(1,-DBL_MAX);
	bounds.second = Q(1,DBL_MAX);
	return bounds;
}

void SteeredWheel::getJacobian(size_t row,
		size_t col,
		const math::Transform3D<>& joint,
		const math::Transform3D<>& tcp,
		const kinematics::State& state,
		math::Jacobian& jacobian) const {
	RevoluteJoint::getJacobian(row,col,joint,tcp,state,jacobian);
}

Transform3D<> SteeredWheel::getFixedTransform() const {
	return RevoluteJoint::getFixedTransform();
}

void SteeredWheel::setFixedTransform( const Transform3D<>& t3d) {
	RevoluteJoint::setFixedTransform(t3d);
}

Transform3D<> SteeredWheel::getJointTransform(const State& state) const {
	return RevoluteJoint::getJointTransform(state);
}

void SteeredWheel::setRadius(double radius) {
	if (_wheel == NULL)
		findWheel();
	_wheel->setRadius(radius);
}

double SteeredWheel::getRadius() {
	if (_wheel == NULL)
		findWheel();
	return _wheel->getRadius();
}

void SteeredWheel::setOffset(double offset) {
	_offset = offset;
}

double SteeredWheel::getOffset() const {
	return _offset;
}

bool SteeredWheel::isStearable() const {
	return true;
}

void SteeredWheel::setWheel(FixedWheel* wheel) {
	_wheel = wheel;
}

FixedWheel* SteeredWheel::getWheel() {
	if (_wheel == NULL)
		findWheel();
	return _wheel;
}

void SteeredWheel::findWheel() {
	BOOST_FOREACH(Frame& child, getChildren()) {
		FixedWheel* wheel = dynamic_cast<FixedWheel*>(&child);
		if (wheel != NULL) {
			if (child.getName().compare(_wheelName) == 0) {
				_wheel = wheel;
			}
			return;
		}
	}
	RW_THROW("No FixedWheel found for SteeredWheel");
}
