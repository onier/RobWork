/*
 * DHParameterCalibration.cpp
 *
 *  Created on: Aug 28, 2012
 *      Author: bing
 */

#include "DHParameterCalibration.hpp"

namespace rwlibs {
namespace calibration {

DHParameterCalibration::DHParameterCalibration(rw::models::Joint::Ptr joint, const Eigen::Vector4d& parameterSet) :
		_joint(joint), _parameterSet(parameterSet), _lockedParameters(Eigen::Vector4i::Zero()) {

}

DHParameterCalibration::~DHParameterCalibration() {

}

rw::models::Joint::Ptr DHParameterCalibration::getJoint() const {
	return _joint;
}

Eigen::Vector4d DHParameterCalibration::getCorrection() const {
	return _parameterSet;
}

void DHParameterCalibration::setCorrection(const Eigen::Vector4d& correction) {
	_parameterSet = correction;
}

void DHParameterCalibration::setLockedParameters(bool a, bool length, bool alpha, bool angle) {
	_lockedParameters << a, length, alpha, angle;
}

void DHParameterCalibration::doApply() {
	rw::models::DHParameterSet dhParameterSetCurrent = *rw::models::DHParameterSet::get(_joint.get());
	double a = dhParameterSetCurrent.a() + _parameterSet(DHPARAMETER_A);
	double alpha = dhParameterSetCurrent.alpha() + _parameterSet(DHPARAMETER_ALPHA);
	if (dhParameterSetCurrent.isParallel()) {
		double b = dhParameterSetCurrent.b() + _parameterSet(DHPARAMETER_B_D);
		double beta = dhParameterSetCurrent.beta() + _parameterSet(DHPARAMETER_BETA_THETA);
		rw::models::DHParameterSet dhParameterSetNext(alpha, a, beta, b, true);
		rw::models::DHParameterSet::set(dhParameterSetNext, _joint.get());
		_joint->setFixedTransform(rw::math::Transform3D<double>::DHHGP(alpha, a, beta, b));
	} else {
		double d = dhParameterSetCurrent.d() + _parameterSet(DHPARAMETER_B_D);
		double theta = dhParameterSetCurrent.theta() + _parameterSet(DHPARAMETER_BETA_THETA);
		rw::models::DHParameterSet dhParameterSetNext(alpha, a, d, theta, dhParameterSetCurrent.getType());
		rw::models::DHParameterSet::set(dhParameterSetNext, _joint.get());
		_joint->setFixedTransform(rw::math::Transform3D<double>::DH(alpha, a, d, theta));
	}
}

void DHParameterCalibration::doRevert() {
	rw::models::DHParameterSet dhParameterSetCurrent = *rw::models::DHParameterSet::get(_joint.get());
	const double a = dhParameterSetCurrent.a() - _parameterSet(DHPARAMETER_A);
	const double alpha = dhParameterSetCurrent.alpha() - _parameterSet(DHPARAMETER_ALPHA);
	if (dhParameterSetCurrent.isParallel()) {
		const double b = dhParameterSetCurrent.b() - _parameterSet(DHPARAMETER_B_D);
		const double beta = dhParameterSetCurrent.beta() - _parameterSet(DHPARAMETER_BETA_THETA);
		rw::models::DHParameterSet dhParameterSetNext(alpha, a, beta, b, true);
		rw::models::DHParameterSet::set(dhParameterSetNext, _joint.get());
		_joint->setFixedTransform(rw::math::Transform3D<double>::DHHGP(alpha, a, beta, b));
	} else {
		const double d = dhParameterSetCurrent.d() - _parameterSet(DHPARAMETER_B_D);
		const double theta = dhParameterSetCurrent.theta() - _parameterSet(DHPARAMETER_BETA_THETA);
		rw::models::DHParameterSet dhParameterSetNext(alpha, a, d, theta, dhParameterSetCurrent.getType());
		rw::models::DHParameterSet::set(dhParameterSetNext, _joint.get());
		_joint->setFixedTransform(rw::math::Transform3D<double>::DH(alpha, a, d, theta));
	}
}

void DHParameterCalibration::doCorrect(rw::kinematics::State& state) {

}

int DHParameterCalibration::doGetParameterCount() const {
	return _lockedParameters.rows() - _lockedParameters.sum();
}

Eigen::MatrixXd DHParameterCalibration::doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame,
		const rw::kinematics::State& state) {
	const Eigen::Affine3d tfmToPreLink = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), _joint->getParent(state), state);
	const Eigen::Affine3d tfmLink = _joint->getFixedTransform();
	const Eigen::Affine3d tfmToPostLink = tfmToPreLink * tfmLink;
	const Eigen::Affine3d tfmJoint = _joint->getJointTransform(state);
	const Eigen::Affine3d tfmPostJoint = rw::kinematics::Kinematics::frameTframe(_joint.get(), targetFrame.get(), state);
	const Eigen::Affine3d tfmToEnd = tfmToPostLink * tfmJoint * tfmPostJoint;

	const bool isParallel = rw::models::DHParameterSet::get(_joint.get())->isParallel();

	const unsigned int columnCount = getParameterCount();
	Eigen::MatrixXd jacobian(6, columnCount);
	int columnIndex = 0;
	// a
	if (!_lockedParameters(DHPARAMETER_A)) {
		jacobian.block<3, 1>(0, columnIndex) = tfmToPostLink.linear().col(0);
		jacobian.block<3, 1>(3, columnIndex++) = Eigen::Vector3d::Zero();
	}
	// b/d
	if (!_lockedParameters(DHPARAMETER_B_D)) {
		jacobian.block<3, 1>(0, columnIndex) = tfmToPreLink.linear().col(isParallel ? 1 : 2);
		jacobian.block<3, 1>(3, columnIndex++) = Eigen::Vector3d::Zero();
	}
	// alpha
	if (!_lockedParameters(DHPARAMETER_ALPHA)) {
		Eigen::Vector3d xAxisToPost = tfmToPostLink.linear().col(0);
		Eigen::Vector3d tlPostToEnd = tfmToEnd.translation() - tfmToPostLink.translation();
		jacobian.block<3, 1>(0, columnIndex) = xAxisToPost.cross(tlPostToEnd);
		jacobian.block<3, 1>(3, columnIndex++) = xAxisToPost;
	}
	// beta/theta
	if (!_lockedParameters(DHPARAMETER_BETA_THETA)) {
		Eigen::Vector3d yzAxisToPre = tfmToPreLink.linear().col(isParallel ? 1 : 2);
		Eigen::Vector3d tlPreToEnd = tfmToEnd.translation() - tfmToPreLink.translation();
		jacobian.block<3, 1>(0, columnIndex) = yzAxisToPre.cross(tlPreToEnd);
		jacobian.block<3, 1>(3, columnIndex) = yzAxisToPre;
	}

	return jacobian;
}

void DHParameterCalibration::doTakeStep(const Eigen::VectorXd& step) {
	const int parameterCount = _lockedParameters.rows();
	unsigned int unlockedParameterIndex = 0;
	Eigen::Vector4d parameterVector = Eigen::Vector4d::Zero();
	for (int parameterIndex = 0; parameterIndex < parameterCount; parameterIndex++)
		if (!_lockedParameters(parameterIndex))
			parameterVector(parameterIndex) = step(unlockedParameterIndex++);

	bool wasApplied = isApplied();
	if (wasApplied)
		revert();

	_parameterSet += parameterVector;

	if (wasApplied)
		apply();
}

}
}
