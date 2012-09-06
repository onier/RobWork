/*
 * DHParameterJacobian.cpp
 *
 *  Created on: Aug 30, 2012
 *      Author: bing
 */

#include "DHParameterJacobian.hpp"

namespace rwlibs {
namespace calibration {

DHParameterJacobian::DHParameterJacobian(DHParameterCalibration::Ptr calibration) :
		_calibration(calibration), _enabledParameters(Eigen::Vector4i::Ones()) {

}

DeviceCalibration::Ptr DHParameterJacobian::getCalibration() const {
	return _calibration;
}

int DHParameterJacobian::getParameterCount() const {
	return _isEnabled ? _enabledParameters.sum() : 0;
}

Eigen::MatrixXd DHParameterJacobian::compute(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
		const rw::kinematics::State& state) {
	if (!_isEnabled)
		RW_THROW("Not enabled.");
	if (!_enabledParameters.sum())
		RW_THROW("No parameters enabled.");

	const rw::models::Joint::Ptr joint = _calibration->getJoint();

	const Eigen::Affine3d tfmToPreLink = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), joint->getParent(state), state);
	const Eigen::Affine3d tfmLink = joint->getFixedTransform();
	const Eigen::Affine3d tfmToPostLink = tfmToPreLink * tfmLink;
	const Eigen::Affine3d tfmJoint = joint->getJointTransform(state);
	const Eigen::Affine3d tfmPostJoint = rw::kinematics::Kinematics::frameTframe(joint.get(), measurementFrame.get(), state);
	const Eigen::Affine3d tfmToEnd = tfmToPostLink * tfmJoint * tfmPostJoint;
	const bool isParallel = rw::models::DHParameterSet::get(joint.get())->isParallel();

	const unsigned int nColumns = _enabledParameters.sum();
	Eigen::MatrixXd jacobian(6, nColumns);
	int columnNo = 0;
	// a
	if (_enabledParameters(0)) {
		jacobian.block<3, 1>(0, columnNo) = tfmToPostLink.linear().col(0);
		jacobian.block<3, 1>(3, columnNo) = Eigen::Vector3d::Zero();
		columnNo++;
	}
	// b/d
	if (_enabledParameters(1)) {
		jacobian.block<3, 1>(0, columnNo) = tfmToPreLink.linear().col(isParallel ? 1 : 2);
		jacobian.block<3, 1>(3, columnNo) = Eigen::Vector3d::Zero();
		columnNo++;
	}
	// alpha
	if (_enabledParameters(2)) {
		Eigen::Vector3d xAxisToPost = tfmToPostLink.linear().col(0);
		Eigen::Vector3d tlPostToEnd = tfmToEnd.translation() - tfmToPostLink.translation();
		jacobian.block<3, 1>(0, columnNo) = xAxisToPost.cross(tlPostToEnd);
		jacobian.block<3, 1>(3, columnNo) = xAxisToPost;
		columnNo++;
	}
	// beta/theta
	if (_enabledParameters(3)) {
		Eigen::Vector3d yzAxisToPre = tfmToPreLink.linear().col(isParallel ? 1 : 2);
		Eigen::Vector3d tlPreToEnd = tfmToEnd.translation() - tfmToPreLink.translation();
		jacobian.block<3, 1>(0, columnNo) = yzAxisToPre.cross(tlPreToEnd);
		jacobian.block<3, 1>(3, columnNo) = yzAxisToPre;
		columnNo++;
	}

	return jacobian;
}

void DHParameterJacobian::step(const Eigen::VectorXd& step) {
	if (!_isEnabled)
		RW_THROW("Not enabled.");
	if (!_enabledParameters.sum())
		RW_THROW("No parameters enabled.");

	unsigned int enabledParameterNo = 0;
	Eigen::Vector4d parameterVector = Eigen::Vector4d::Zero();
	for (int parameterNo = 0; parameterNo < _enabledParameters.rows(); parameterNo++)
		if (_enabledParameters(parameterNo)) {
			parameterVector(parameterNo) = step(enabledParameterNo);
			enabledParameterNo++;
		}

	double a = parameterVector(0), alpha = parameterVector(2);
	if (_calibration->getCorrection().isParallel()) {
		double b = parameterVector(1), beta = parameterVector(3);
		_calibration->correct(rw::models::DHParameterSet(alpha, a, beta, b, true));
	} else {
		double d = parameterVector(1), theta = parameterVector(3);
		_calibration->correct(rw::models::DHParameterSet(alpha, a, d, theta, _calibration->getCorrection().getType()));
	}
}

void DHParameterJacobian::setEnabledParameters(bool a, bool length, bool alpha, bool angle) {
	_enabledParameters << a, length, alpha, angle;
}

}
}
