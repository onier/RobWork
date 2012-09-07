/*
 * EncoderParameterJacobian.cpp
 *
 *  Created on: Aug 30, 2012
 *      Author: bing
 */

#include "EncoderParameterJacobian.hpp"

namespace rwlibs {
namespace calibration {

EncoderParameterJacobian::EncoderParameterJacobian(rw::models::SerialDevice::Ptr serialDevice, EncoderParameterCalibration::Ptr calibration) :
		_serialDevice(serialDevice), _calibration(calibration), _enabledParameters(Eigen::Vector2i::Ones()) {
	// Find joint number.
	const rw::models::Joint::Ptr joint = _calibration->getJoint();
	const std::vector<rw::models::Joint*> joints = _serialDevice->getJoints();
	_jointNo = std::find(joints.begin(), joints.end(), joint.get()) - joints.begin();

}

DeviceCalibration::Ptr EncoderParameterJacobian::getCalibration() const {
	return _calibration;
}

int EncoderParameterJacobian::getParameterCount() const {
	return isEnabled() ? _enabledParameters.sum() : 0;
}

Eigen::MatrixXd EncoderParameterJacobian::compute(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
		const rw::kinematics::State& state) {
	if (!isEnabled())
		RW_THROW("Not enabled.");
	if (!_enabledParameters.sum())
		RW_THROW("No parameters enabled.");

	// Get joint value.
	const rw::math::Q q = _serialDevice->getQ(state);
	const double qi = q[_jointNo];

	// Prepare transformations.
	const rw::models::Joint::Ptr joint = _calibration->getJoint();
	const Eigen::Affine3d tfmToPostJoint = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), joint.get(), state);
	const Eigen::Affine3d tfmPostJoint = rw::kinematics::Kinematics::frameTframe(joint.get(), measurementFrame.get(), state);
	const Eigen::Affine3d tfmToEnd = tfmToPostJoint * tfmPostJoint;
	const Eigen::Vector3d posToEnd = tfmToEnd.translation() - tfmToPostJoint.translation();
	const Eigen::Vector3d jointAxis = tfmToPostJoint.linear().col(2);

	const unsigned int nColumns = _enabledParameters.sum();
	Eigen::MatrixXd jacobian(6, nColumns);
	int columnNo = 0;
	// tau
	if (_enabledParameters(0)) {
		jacobian.block<3, 1>(0, columnNo) = -sin(qi) * jointAxis.cross(posToEnd);
		jacobian.block<3, 1>(3, columnNo) = -sin(qi) * jointAxis;
		columnNo++;
	}
	// sigma
	if (_enabledParameters(1)) {
		jacobian.block<3, 1>(0, columnNo) = -cos(qi) * jointAxis.cross(posToEnd);
		jacobian.block<3, 1>(3, columnNo) = -cos(qi) * jointAxis;
		columnNo++;
	}

	return jacobian;
}

void EncoderParameterJacobian::step(const Eigen::VectorXd& step) {
	if (!isEnabled())
		RW_THROW("Not enabled.");
	if (!_enabledParameters.sum())
		RW_THROW("No parameters enabled.");

	unsigned int enabledParameterNo = 0;
	Eigen::Vector2d parameterVector = Eigen::Vector2d::Zero();
	for (int parameterNo = 0; parameterNo < _enabledParameters.rows(); parameterNo++)
		if (_enabledParameters(parameterNo)) {
			parameterVector(parameterNo) = step(enabledParameterNo);
			enabledParameterNo++;
		}

	_calibration->correct(parameterVector);
}

void EncoderParameterJacobian::setEnabledParameters(bool tau, bool sigma) {
	_enabledParameters << tau, sigma;
}

}
}
