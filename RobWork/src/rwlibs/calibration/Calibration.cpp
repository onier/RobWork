/*
 * Calibration.cpp
 *
 *  Created on: Sep 13, 2012
 *      Author: bing
 */

#include "Calibration.hpp"

namespace rwlibs {
namespace calibration {

Calibration::~Calibration() {

}

bool Calibration::isLocked() const {
	return _isLocked;
}

void Calibration::setLocked(bool isLocked) {
	if (_isApplied)
		RW_THROW("Already applied.");

	_isLocked = isLocked;
}

bool Calibration::isApplied() const {
	return _isApplied;
}

void Calibration::apply() {
	if (_isLocked)
		RW_THROW("Locked.");
	if (_isApplied)
		RW_THROW("Already applied.");

	doApply();

	_isApplied = true;
}

void Calibration::revert() {
	if (_isLocked)
		RW_THROW("Locked.");
	if (!_isApplied)
		RW_THROW("Not applied.");

	doRevert();

	_isApplied = false;
}

void Calibration::correct(rw::kinematics::State& state) {
	if (_isLocked)
		RW_THROW("Locked.");
	if (!_isApplied)
		RW_WARN("Not applied.");

	doCorrect(state);
}

int Calibration::getParameterCount() const {
	return _isLocked ? 0 : doGetParameterCount();
}

Eigen::MatrixXd Calibration::computeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& state) {
	if (_isLocked)
		RW_THROW("Locked.");
	if (doGetParameterCount() == 0)
		RW_THROW("No parameters enabled.");

	return doComputeJacobian(referenceFrame, measurementFrame, state);
}

void Calibration::step(const Eigen::VectorXd& step) {
	if (_isLocked)
		RW_THROW("Locked.");
	if (doGetParameterCount() == 0)
		RW_THROW("No parameters enabled.");

	return doStep(step);
}

Calibration::Calibration() :
		_isLocked(false), _isApplied(false) {
}

}
}
