/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "SpringJointController.hpp"

#include <rw/common/macros.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rwlibs::control;
using namespace rwlibs::simulation;
using namespace rwsim::control;
using namespace rwsim::dynamics;

SpringJointController::SpringJointController(
        const std::string& name,
		RigidDevice::Ptr rdev,
		const std::vector<SpringParam>& springParam):
	JointController(name, &rdev->getModel()),
	SimulatedController(rw::common::ownedPtr(new rw::models::ControllerModel(name,rdev->getKinematicModel()->getBase()))),
	_ddev(rdev),
	_target(Q::zero(rdev->getModel().getDOF())),
	_currentQ(_target),
	_springParams(springParam),
	_enabled(true)
{
}

void SpringJointController::setTargetPos(const Q& target) {
    _target = target;
}

void SpringJointController::setTargetVel(const Q& vals) {
	RW_THROW("SpringJointController (setTargetVel): this type of control is unsupported!");
}

void SpringJointController::setTargetAcc(const Q& vals) {
	RW_THROW("SpringJointController (setTargetAcc): this type of control is unsupported!");
}

void SpringJointController::update(const Simulator::UpdateInfo& info, State& state) {
    const Q q = _ddev->getModel().getQ(state);
    Q q_error = _currentQ-q;
    if(info.rollback){
        // then we use the last calculated error
        q_error = _qError;
    } else {
        _qError = q_error;
    }

    // the error in configuration result in a torque
    Q torque(q_error.size());
    if(info.dt_prev>0.0){
        for(size_t i=0; i<torque.size(); i++) {
            torque(i) = (-_target[i]+q(i)+_springParams[i].offset)*_springParams[i].elasticity - (q_error(i)*_springParams[i].dampening)/info.dt_prev;
            std::cout << "spring joint " << i << ": " << -_target[i]+q(i)+_springParams[i].offset << std::endl;
        }
    } else {
        for(size_t i=0; i<torque.size(); i++)
            torque(i) = (-_target[i]+q(i)+_springParams[i].offset)*_springParams[i].elasticity;
    }

    std::cout << "spring joint apply: " << torque << std::endl;
    _ddev->setMotorForceTargets(torque, state);

    _currentQ = q;
}

void SpringJointController::reset(const State& state) {
    _currentQ = _ddev->getModel().getQ(state);
    _target = _currentQ;
}

void SpringJointController::setControlMode(ControlMode mode) {
    if(mode != POSITION)
        RW_THROW("SpringJointController only supports the positional control mode!");
}

unsigned int SpringJointController::getControlModes() {
	return POSITION;
}
