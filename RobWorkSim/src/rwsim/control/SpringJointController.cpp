#include "SpringJointController.hpp"

#include <rw/common/macros.hpp>

using namespace rwsim::control;
using namespace rwsim::dynamics;
using namespace rw::math;

namespace {

}

SpringJointController::SpringJointController(
        const std::string& name,
        DynamicDevice* rdev,
		const std::vector<SpringParam>& springParam,
		double dt):
	JointController(name, &rdev->getModel()),
	_ddev(rdev),
	_lastError(rw::math::Q::zero(rdev->getModel().getDOF())),
	_target(rw::math::Q::zero(rdev->getModel().getDOF())),
	_currentQ(_target),
	_currentVel(rw::math::Q::zero(_target.size())),
	_targetVel(rw::math::Q::zero(_target.size())),
	_stime(dt),
	_springParams(springParam)
{

}

void SpringJointController::setTargetPos(const rw::math::Q& target){
    _target = target;
}

void SpringJointController::setTargetVel(const rw::math::Q& vals){
	_targetVel = vals;
}

void SpringJointController::setTargetAcc(const rw::math::Q& vals){};


double SpringJointController::getSampleTime(){
	return _stime;
}

void SpringJointController::setSampleTime(double stime){
	_stime = stime;
}

void SpringJointController::update(double dt, rw::kinematics::State& state) {
    // all joints in the device are dependent on a single input

    // the pressure indicate the size of the torques that are applied on the beam joints.
    // the closer the beamjoints are at thier resting configuration the smaller torque
	Q q = _ddev->getModel().getQ(state);
	Q q_error = _currentQ-q;

	// the error in configuration result in a torque
	Q torque(q_error.size());
	for(size_t i=0; i<torque.size(); i++)
	    torque(i) = (q(i)+_springParams[i].offset)*_springParams[i].elasticity - (q_error(i)*_springParams[i].dampening)/dt;

	std::cout << "TORQUE\n" << torque << std::endl;

	_ddev->addForceTorque(torque, state);
	//_ddev->setForceLimit(torque);
	//_ddev->setVelocity(q_error, state);

	_currentQ = q;
}

void SpringJointController::reset(const rw::kinematics::State& state){
    _currentQ = _ddev->getModel().getQ(state);
    _target = _currentQ;
    _targetVel = rw::math::Q::zero(_currentQ.size());
    //_time = 0;
}

void SpringJointController::setControlMode(ControlMode mode){
    if(mode!=POSITION || mode !=VELOCITY )
        RW_THROW("Unsupported control mode!");
    _mode = mode;
}
