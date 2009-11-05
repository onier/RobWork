/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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


#include "Conveyor.hpp"

#include <rw/models/BasicDeviceJacobian.hpp>
#include <rw/math/Jacobian.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

namespace
{
	std::vector<Joint*> constructJointList(Joint* joint)
    {
        return std::vector<Joint*>(1, joint);
	}
}

Conveyor::Conveyor(
    const std::string& name,
    FixedJoint* base,
    const std::vector<ConveyorSegment*>& segments)
    :
	Device(name),
    _segments(segments),
    _base(base),
    _basicDevice(constructJointList(base))
{
    typedef std::vector<ConveyorSegment*>::const_iterator I;
    for (I it = _segments.begin(); it != _segments.end(); ++it) {
        _frame2segment[(*it)->getBaseFrame()] = *it;
    }
}

void Conveyor::addItem(ConveyorItem* item, double time, State& state) {
	_segments.front()->addItem(item, ConveyorSegment::START, state);
	_segments.front()->move(item, time, state);
}

void Conveyor::setQ(const Q& q, State& state) const {
    double qcurrent = _basicDevice.getQ(state)(0);
    double delta = q(0)-qcurrent;

    //TODO Construct List of all items
    std::vector<ConveyorItem*> items;
    std::vector<ConveyorSegment*>::const_iterator it = _segments.begin();
    for ( ; it != _segments.end(); ++it) {
    	Frame::const_iterator_pair pair = (*it)->getBaseFrame()->getDafChildren(state);
    	for (Frame::const_iterator itframe = pair.first; itframe != pair.second; ++itframe) {
    		if (dynamic_cast<const ConveyorItem*>(&(*itframe)) != NULL)
    			items.push_back((ConveyorItem*)(&(*itframe)));
    	}
    }

    //TODO move frames
    std::map<rw::kinematics::Frame*, ConveyorSegment*>& map =
        const_cast<std::map<Frame*, ConveyorSegment*>&>(_frame2segment);

    for (std::vector<ConveyorItem*>::iterator it = items.begin(); it != items.end(); ++it) {
    	ConveyorSegment* segment = map[(*it)->getParent(state)];
    	segment->move((*it), delta, state);
    }

    _basicDevice.setQ(q, state);
}

Q Conveyor::getQ(const State& state) const {
	return _basicDevice.getQ(state);
}

std::pair<Q, Q> Conveyor::getBounds() const {
	return _basicDevice.getBounds();
}

void Conveyor::setBounds(const std::pair<Q, Q>& bounds) {
	_basicDevice.setBounds(bounds);
}

Q Conveyor::getVelocityLimits() const {
	return _basicDevice.getVelocityLimits();
}

void Conveyor::setVelocityLimits(const Q& vellimits) {
	_basicDevice.setVelocityLimits(vellimits);
}

Q Conveyor::getAccelerationLimits() const {
	return _basicDevice.getAccelerationLimits();
}

void Conveyor::setAccelerationLimits(const Q& acclimits) {
	_basicDevice.setAccelerationLimits(acclimits);
}

size_t Conveyor::getDOF() const { return _basicDevice.getDOF(); }

Frame* Conveyor::getBase() { return _base; }

const Frame* Conveyor::getBase() const { return _base; }

Frame* Conveyor::getEnd() { return _base; }

const Frame* Conveyor::getEnd() const { return _base; }

Jacobian Conveyor::baseJend(const State& state) const
{
    return baseJframe(getEnd(), state);
}

Jacobian Conveyor::baseJframe(const Frame* frame, const State& state) const
{
	BasicDeviceJacobian jac(_basicDevice, getBase(), frame, state);
	return jac.get(state);
}

Jacobian Conveyor::baseJframes(
    const std::vector<Frame*>& frames,
    const State& state) const
{
	BasicDeviceJacobian jac(_basicDevice, getBase(), frames, state);
	return jac.get(state);
}

boost::shared_ptr<DeviceJacobian> Conveyor::baseDJframes(
    const std::vector<Frame*>& frames,
    const State& state) const
{
    typedef boost::shared_ptr<DeviceJacobian> T;
    return T(
        new BasicDeviceJacobian(
            _basicDevice,
            getBase(),
            frames,
            state));
}
