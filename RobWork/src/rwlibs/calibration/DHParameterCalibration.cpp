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
_joint(joint), _correction(parameterSet), _originalSet(*rw::models::DHParameterSet::get(joint.get())) {

}

DHParameterCalibration::~DHParameterCalibration() {

}

rw::models::Joint::Ptr DHParameterCalibration::getJoint() const {
	return _joint;
}

Eigen::Vector4d DHParameterCalibration::getCorrection() const {
	return _correction;
}

void DHParameterCalibration::setCorrection(const Eigen::Vector4d& correction) {
	_correction = correction;
}

void DHParameterCalibration::doApply() {
	_originalSet = *rw::models::DHParameterSet::get(_joint.get());
	double a = _originalSet.a() + _correction(PARAMETER_A);
	double alpha = _originalSet.alpha() + _correction(PARAMETER_ALPHA);
	if (_originalSet.isParallel()) {
		double b = _originalSet.b() + _correction(PARAMETER_B);
		double beta = _originalSet.beta() + _correction(PARAMETER_BETA);
		rw::models::DHParameterSet correctedSet(alpha, a, beta, b, true);
		rw::models::DHParameterSet::set(correctedSet, _joint.get());
		_joint->setFixedTransform(rw::math::Transform3D<double>::DHHGP(alpha, a, beta, b));
	} else {
		double d = _originalSet.d() + _correction(PARAMETER_D);
		double theta = _originalSet.theta() + _correction(PARAMETER_THETA);
		rw::models::DHParameterSet correctedSet(alpha, a, d, theta, _originalSet.getType());
		rw::models::DHParameterSet::set(correctedSet, _joint.get());
		_joint->setFixedTransform(rw::math::Transform3D<double>::DH(alpha, a, d, theta));
	}
}

void DHParameterCalibration::doRevert() {
	if (_originalSet.isParallel()) {
		rw::models::DHParameterSet::set(_originalSet, _joint.get());
		_joint->setFixedTransform(rw::math::Transform3D<double>::DHHGP(_originalSet.alpha(), _originalSet.a(), _originalSet.beta(), _originalSet.b()));
	} else {
		rw::models::DHParameterSet::set(_originalSet, _joint.get());
		_joint->setFixedTransform(rw::math::Transform3D<double>::DH(_originalSet.alpha(), _originalSet.a(), _originalSet.d(), _originalSet.theta()));
	}
}

void DHParameterCalibration::doCorrectState(rw::kinematics::State& state) {

}

}
}
