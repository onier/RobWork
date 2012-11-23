/*
 * JacobianBase.cpp
 *
 *  Created on: Nov 22, 2012
 *      Author: bing
 */

#include "JacobianBase.hpp"

namespace rwlibs {
namespace calibration {

JacobianBase::~JacobianBase() {

}

bool JacobianBase::isEnabled() const {
	return _isEnabled;
}

void JacobianBase::setEnabled(bool isEnabled) {
	_isEnabled = isEnabled;
}

int JacobianBase::getParameterCount() const {
	return isEnabled() ? _enabledParameters.sum() : 0;
}

bool JacobianBase::isParameterEnabled(int parameterIndex) {
	RW_ASSERT(parameterIndex < _enabledParameters.size());
	return _enabledParameters(parameterIndex);
}

void JacobianBase::setParameterEnabled(int parameterIndex, bool isEnabled) {
	RW_ASSERT(parameterIndex < _enabledParameters.size());
	_enabledParameters(parameterIndex) = isEnabled;
}

Eigen::MatrixXd JacobianBase::computeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame, const rw::kinematics::State& state) {
	RW_ASSERT(getParameterCount() != 0);
	return doComputeJacobian(referenceFrame, targetFrame, state);
}

void JacobianBase::takeStep(const Eigen::VectorXd& step) {
	RW_ASSERT(getParameterCount() != 0);

	// Map step.
	const int parameterCount = _enabledParameters.rows();
	unsigned int enabledParameterIndex = 0;
	Eigen::VectorXd mappedStep = Eigen::VectorXd::Zero(parameterCount);
	for (int parameterIndex = 0; parameterIndex < parameterCount; parameterIndex++)
		if (isParameterEnabled(parameterIndex))
			mappedStep(parameterIndex) = step(enabledParameterIndex++);

	return doTakeStep(mappedStep);
}

JacobianBase::JacobianBase(int parameterCount) :
		_isEnabled(true), _enabledParameters(Eigen::VectorXi::Ones(parameterCount)) {

}

}
}
