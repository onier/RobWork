#include "PDController.hpp"

#include <rw/common/macros.hpp>

using namespace rwsim::control;
using namespace rwsim::dynamics;

PDController::PDController(
		RigidDevice* rdev, const rw::kinematics::State& state,
		ControlMode cmode,
		const std::vector<PDParam>& pdparams,
		double dt):
	JointController(&rdev->getModel()),
	_ddev(rdev),
	_lastError(rw::math::Q::zero(rdev->getModel().getDOF())),
	_target(rdev->getModel().getQ(state)),
	_currentQ(_target),_currentVel(rw::math::Q::zero(_target.size())),
	_targetVel(rw::math::Q::zero(_target.size())),
	_pdparams(pdparams),
	_mode(cmode),
	_stime(dt)
{
	if(pdparams.size()!=_ddev->getModel().getDOF())
		RW_THROW("Nr of PDParams must match the nr of DOF in the Device!");
}

PDController::PDController(
		RigidDevice* rdev, const rw::kinematics::State& state,
		ControlMode cmode,
		const PDParam& pdparam,
		double dt):
	JointController(&rdev->getModel()),
	_ddev(rdev),
	_lastError(rw::math::Q::zero(rdev->getModel().getDOF())),
	_target(rdev->getModel().getQ(state)),
	_currentQ(_target),_currentVel(rw::math::Q::zero(_target.size())),
	_targetVel(rw::math::Q::zero(_target.size())),
	_pdparams(rdev->getModel().getDOF(),pdparam),
	_mode(cmode),
	_stime(dt)
{
}

std::vector<PDParam> PDController::getParameters(){
	return _pdparams;
}

void PDController::setParameters(const std::vector<PDParam>& params){
	if(params.size()!=_ddev->getModel().getDOF())
		RW_THROW("Nr of PDParams must match the nr of DOF in the Device!");
	_pdparams = params;
}

double PDController::getSampleTime(){
	return _stime;
}

void PDController::setSampleTime(double stime){
	_stime = stime;
}

void PDController::update(double dt, rw::kinematics::State& state) {
	_accTime+=dt;
	if(_accTime>_stime){

		const double P = 10;
		const double D = 0.3;
		rw::math::Q q = _ddev->getModel().getQ(state);
		rw::math::Q error = _target-q;
		// std::cout  << "PD TARGET: " << _target << std::endl;
		// std::cout  << "PD ERROR: " << error << std::endl;
		rw::math::Q nvel = P*error + ((error-_lastError)/_accTime)*D;
		_lastError = error;

		_ddev->setVelocity(_targetVel + nvel, state);

		_currentVel = (q - _currentQ)/dt;
		_currentQ = q;

		_accTime -= _stime;
	}
}

void PDController::reset(const rw::kinematics::State& state){
    _currentQ = _ddev->getModel().getQ(state);
    _target = _currentQ;
    _targetVel = rw::math::Q::zero(_currentQ.size());
    //_time = 0;
}

void PDController::setControlMode(ControlMode mode){
    if(mode!=POSITION || mode !=VELOCITY )
        RW_THROW("Unsupported control mode!");
    _mode = mode;
}

