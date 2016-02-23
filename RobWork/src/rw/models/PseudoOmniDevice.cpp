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

#include "PseudoOmniDevice.hpp"

#include <rw/math/RPY.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/common/macros.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

namespace {
std::vector<Frame*> ends(const std::vector<SteeredWheel*> &steeredWheels) {
	std::vector<Frame*> ends;
	for (std::vector<SteeredWheel*>::const_iterator it = steeredWheels.begin(); it < steeredWheels.end(); it++) {
		ends.push_back((*it)->getWheel());
	}
	return ends;
}

Q normalizeQ(const Q q) {
	Q qAdj = q;
	for (std::size_t i = 0; i < qAdj.size(); i++) {
		while (qAdj[i] > Pi) qAdj[i] -= 2*Pi;
		while (qAdj[i] < -Pi) qAdj[i] += 2*Pi;
	}
	return qAdj;
}

double normalizeQ(double q) {
	double qAdj = q;
	while (qAdj > Pi) qAdj -= 2*Pi;
	while (qAdj < -Pi) qAdj += 2*Pi;
	return qAdj;
}

double angle(Vector3D<> v1, Vector3D<> v2, Vector3D<> normal) {
	return dot(cross(v1-dot(v1,normal)*normal,v2-dot(v2,normal)*normal),normal);
}
}

PseudoOmniDevice::PseudoOmniDevice(
    MovableFrame* base,
    const std::vector<SteeredWheel*> &steeredWheels,
    State& state,
    const std::string& name)
    :
    Device(name),
    _base(base),
    _wheels(steeredWheels),
    _device(base, ends(steeredWheels), name, state),
    _mode(STOPPED),
    _wheel1(0),
    _wheel2(1)
{
    setDevicePose(Transform3D<>::identity(), state);
}

PseudoOmniDevice::~PseudoOmniDevice() {}

void PseudoOmniDevice::setDevicePose(const Transform3D<>& transform, State& state)
{
    _base->setTransform(transform, state);
}

void PseudoOmniDevice::setQ(const Q& qraw, State& state) const
{
	const Q q = normalizeQ(qraw);
	const Q qold = _device.getQ(state);
	const Q dq = normalizeQ(q-qold);
	const double radius = _wheels[0]->getRadius();
	const double offset = _wheels[0]->getOffset();
	const double couplingFactor = offset/radius;

	if (_mode == STOPPED) {
		std::size_t iMaxSteer = 0;
		for (std::size_t i = 0; i < dq.size(); i += 2) {
			if (fabs(dq[i]) > fabs(dq[iMaxSteer]))
				iMaxSteer = i;
		}
		Q newQ = qold;
		newQ[iMaxSteer] = q[iMaxSteer];
		newQ[iMaxSteer+1] += couplingFactor*dq[iMaxSteer];
		_device.setQ(normalizeQ(newQ),state);
	} else if (_mode == STRAIGHT) {
		std::size_t iMaxSteer = 0;
		std::size_t iMaxWheel = 1;
		for (std::size_t i = 0; i < dq.size(); i += 2) {
			if (fabs(dq[i]) > fabs(dq[iMaxSteer]))
				iMaxSteer = i;
			if (fabs(dq[i+1]) > fabs(dq[iMaxWheel]))
				iMaxWheel = i+1;
		}
		Q newQ = qold;
		for (std::size_t i = 0; i < dq.size(); i += 2) {
			newQ[i] = q[iMaxSteer];
			newQ[i+1] += couplingFactor*normalizeQ(newQ[i]-qold[i]);
			newQ[i+1] += dq[iMaxWheel];
		}
		_device.setQ(normalizeQ(newQ),state);

		Transform3D<> transform = _base->getTransform(state);
		double theta = dq[iMaxWheel];
		Vector3D<> vec = _wheels[0]->getTransform(state).R().getCol(0);
		transform.P() += vec*theta*radius;
		_base->setTransform(transform, state);
	} else if (_mode == TURN) {
		Q newQ = qold;
		// Steer the two active wheels
		newQ[_wheel1*2] += dq[_wheel1*2];
		newQ[_wheel2*2] += dq[_wheel2*2];
		newQ[_wheel1*2+1] += couplingFactor*normalizeQ(dq[_wheel1*2]);
		newQ[_wheel2*2+1] += couplingFactor*normalizeQ(dq[_wheel2*2]);
		_device.setQ(normalizeQ(newQ),state);
		// Find ICM if possible
		Vector3D<> icm;
		if (findICM(icm, state)) {
			std::size_t iMaxWheel = 1;
			for (std::size_t i = 0; i < dq.size(); i += 2) {
				if (fabs(dq[i+1]) > fabs(dq[iMaxWheel]))
					iMaxWheel = i+1;
			}
			Transform3D<> maxT = _wheels[iMaxWheel/2]->getParent()->getTransform(state)*_wheels[iMaxWheel/2]->getTransform(state);
			Vector3D<> maxNewDir = icm-maxT.P();
			double maxWheelRadius = maxNewDir.norm2();
			double inv;
			if (dot(maxT.R().getCol(1),maxNewDir) > 0)
				inv = 1.0;
			else
				inv = -1.0;
			for (std::size_t i = 0; i < _wheels.size(); i++) {
				Transform3D<> T = _wheels[i]->getParent()->getTransform(state)*_wheels[i]->getTransform(state);
				Vector3D<> newDir = icm-T.P();
				double wheelRadius = newDir.norm2();
				if (i != _wheel1 && i != _wheel2) {
					if (wheelRadius > 1e-3) { // 1 mm
						Vector3D<> oldDir = T.R().getCol(1);
						newQ[i*2] += angle(oldDir,normalize(newDir),T.R().getCol(2));
						newQ[i*2+1] += couplingFactor*normalizeQ(newQ[i*2]-qold[i*2]);
					}
				}
				if (maxWheelRadius > 1e-3) { // 1 mm
					if (dot(T.R().getCol(1),newDir) > 0)
						newQ[i*2+1] += inv*dq[iMaxWheel]*wheelRadius/maxWheelRadius;
					else
						newQ[i*2+1] -= inv*dq[iMaxWheel]*wheelRadius/maxWheelRadius;
				}
			}
			_device.setQ(normalizeQ(newQ),state);

			Transform3D<> transform = _base->getTransform(state);
			Transform3D<> wheelT = transform*_wheels[iMaxWheel/2]->getParent()->getTransform(state)*_wheels[iMaxWheel/2]->getTransform(state);
			double theta = dq[iMaxWheel];
			double turnRadius = (icm-wheelT.P()).norm2();
			if (turnRadius > 1e-3) {
				//Vector3D<> vec = cross(transform.R().getCol(2),icm-transform.P());
				double angle = theta*radius/(turnRadius*2*Pi)*2*Pi;
				Rotation3D<> rot = EAA<>(transform.R().getCol(2),angle).toRotation3D();
				transform.P() = icm+rot*(transform.P()-icm);
				transform.R() = transform.R()*rot;
			}
			_base->setTransform(transform, state);
		} else {
			// Do parallel drive
			std::size_t iMaxWheel = 1;
			for (std::size_t i = 0; i < dq.size(); i += 2) {
				if (fabs(dq[i+1]) > fabs(dq[iMaxWheel]))
					iMaxWheel = i+1;
			}
			Q newQ = qold;
			for (std::size_t i = 0; i < dq.size(); i += 2) {
				newQ[i+1] += dq[iMaxWheel];
			}
			_device.setQ(normalizeQ(newQ),state);

			Transform3D<> transform = _base->getTransform(state);
			double theta = dq[iMaxWheel];
			Vector3D<> vec = _wheels[0]->getTransform(state).R().getCol(0);
			transform.P() += vec*theta*radius;
			_base->setTransform(transform, state);
		}
	}
}

Q PseudoOmniDevice::getQ(const State& state) const
{
    return _device.getQ(state);
}

std::pair<Q, Q> PseudoOmniDevice::getBounds() const
{
    return _device.getBounds();
}

void PseudoOmniDevice::setBounds(const std::pair<Q, Q>& bounds)
{
	_device.setBounds(bounds);
}

Q PseudoOmniDevice::getVelocityLimits() const
{
    return _device.getVelocityLimits();
}

void PseudoOmniDevice::setVelocityLimits(const Q& velLimits)
{
	_device.setVelocityLimits(velLimits);
}

Q PseudoOmniDevice::getAccelerationLimits() const
{
    return _device.getAccelerationLimits();
}

void PseudoOmniDevice::setAccelerationLimits(const Q& accLimits)
{
	_device.setAccelerationLimits(accLimits);
}

size_t PseudoOmniDevice::getDOF() const {
	size_t dof = 0;
	for (std::vector<SteeredWheel*>::const_iterator it = _wheels.begin(); it < _wheels.end(); it++) {
		dof += 2;
	}
	return dof;
}

Frame* PseudoOmniDevice::getBase() {
	return _base;
}

const Frame* PseudoOmniDevice::getBase() const {
	return _base;
}

Frame* PseudoOmniDevice::getEnd() {
    if (!_axillaryFrames.empty())
        return _axillaryFrames.back();
    return _base;
}

const Frame* PseudoOmniDevice::getEnd() const {
    if (!_axillaryFrames.empty())
        return _axillaryFrames.back();

    return _base;
}

Jacobian PseudoOmniDevice::baseJend(const State& state) const {
	double _width = 0.1;
    double dtheta1 = -1/_width;
    double dtheta2 = 1/_width;

    Jacobian jac(Jacobian::zero(6,2));
    jac(0,0) = 0.5;
    jac(1,0) = 0.5;
    jac(5,0) = dtheta1;
    jac(5,1) = dtheta2;

    return jac;
}

Jacobian PseudoOmniDevice::baseJframe(
    const Frame* frame,
    const State& state) const
{
    RW_THROW("Not implemented.");
    return Jacobian(Jacobian::zero(6,2));
}

Jacobian PseudoOmniDevice::baseJframes(
    const std::vector<Frame*>& frames,
    const State& state) const
{
    RW_THROW("Not implemented.");
    return Jacobian(Jacobian::zero(6,2));
}

JacobianCalculatorPtr PseudoOmniDevice::baseJCframes(const std::vector<Frame*>& frames,
                                                 const State& state) const
{
    RW_THROW("Not implemented.");
    return NULL;
}

PseudoOmniDevice::MODE PseudoOmniDevice::getMode() const {
	return _mode;
}

void PseudoOmniDevice::setMode(MODE mode, State &state) {
	const Q q =  _device.getQ(state);
	const double radius = _wheels[0]->getRadius();
	const double offset = _wheels[0]->getOffset();
	const double couplingFactor = offset/radius;

	if (_mode == STOPPED) {
		if (mode == STRAIGHT) {
			double avg = 0;
			for (std::size_t i = 0; i < q.size(); i += 2)
				avg += q[i];
			avg /= q.size();

			Q newQ = q;
			for (std::size_t i = 0; i < q.size(); i += 2) {
				newQ[i] = avg;
				newQ[i+1] += couplingFactor*(avg-q[i]);
			}
			_device.setQ(newQ,state);

			_mode = mode;
		} else if (mode == TURN) {
			_mode = mode;
		}
	} else if (_mode == STRAIGHT) {
		if (mode == STOPPED) {
			_mode = mode;
		} else if (mode == TURN) {
			_mode = mode;
		}
	} else if (_mode == TURN) {
		if (mode == STOPPED) {
			_mode = mode;
		} else if (mode == STRAIGHT) {
			_mode = mode;
		}
	}
}

std::vector<std::string> PseudoOmniDevice::getControlls() const {
	std::vector<std::string> vec;
	for (std::size_t i = 0; i < getDOF(); i+= 2) {
		std::stringstream str;
		str << "q" << i;
		vec.push_back(str.str());
	}
	return vec;
}

void PseudoOmniDevice::setControlls(std::size_t control1, std::size_t control2, State &state) {
	_wheel1 = control1;
	_wheel2 = control2;
	setMode(TURN,state);
}

bool PseudoOmniDevice::findICM(Vector3D<> &icm, const State &state) const {
	if (_mode != TURN)
		return false;
	Transform3D<> T1 = _wheels[_wheel1]->getParent()->getTransform(state)*_wheels[_wheel1]->getTransform(state);
	Transform3D<> T2 = _wheels[_wheel2]->getParent()->getTransform(state)*_wheels[_wheel2]->getTransform(state);
	const double maxRadius = 1000.0; // meters
	const Vector3D<> rotDir = T1.R().getCol(2);
	const Vector3D<> refDir = T1.P()-T2.P();
	const Vector3D<> wheel1Dir = T1.R().getCol(1);
	const Vector3D<> wheel2Dir = T2.R().getCol(1);
	const double sin1 = angle(-refDir,wheel1Dir,rotDir);
	const double sin2 = angle(refDir,wheel2Dir,rotDir);
	const double sinFixed1 = angle(T2.P()-T1.P(),-T1.P(),rotDir);
	const double radius1 = sin2/angle(wheel1Dir,wheel2Dir,rotDir);
	const double wheelDist1 = T1.P().norm2();
	if (radius1 <= maxRadius/* || radius2 <= maxRadius*/) {
		double radius = pow(radius1,2)+pow(wheelDist1,2)-radius1*wheelDist1*cos(asin(sin1) + asin(sinFixed1));
		if (radius <= maxRadius) {
			icm = T1.P()+radius*wheel1Dir;
			icm = T1.P()-angle(refDir,wheel2Dir,rotDir)/angle(wheel1Dir,wheel2Dir,rotDir)*wheel1Dir;
			return true;
		}
	}
	return false;
}
