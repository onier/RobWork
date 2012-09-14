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

bool Calibration::isEnabled() const {
	return _isEnabled;
}

void Calibration::setEnabled(bool isEnabled) {
	if (_isApplied)
		RW_THROW("Already applied.");

	_isEnabled = isEnabled;
}

bool Calibration::isApplied() const {
	return _isApplied;
}

void Calibration::apply() {
	if (!_isEnabled)
		RW_THROW("Not enabled.");
	if (_isApplied)
		RW_THROW("Already applied.");

	doApply();

	_isApplied = true;
}

void Calibration::revert() {
	if (!_isEnabled)
		RW_THROW("Not enabled.");
	if (!_isApplied)
		RW_THROW("Not applied.");

	doRevert();

	_isApplied = false;
}

void Calibration::correct(rw::kinematics::State& state) {
	if (!_isEnabled)
		RW_THROW("Not enabled.");
	if (!_isApplied)
		RW_WARN("Not applied.");

	doCorrect(state);
}

int Calibration::getParameterCount() const {
	return _isEnabled ? doGetParameterCount() : 0;
}

Eigen::MatrixXd Calibration::compute(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& state) {
	if (!_isEnabled)
		RW_THROW("Not enabled.");
	if (!doGetParameterCount())
		RW_THROW("No parameters enabled.");

	return doCompute(referenceFrame, measurementFrame, state);
}

void Calibration::step(const Eigen::VectorXd& step) {
	if (!_isEnabled)
		RW_THROW("Not enabled.");
	if (!doGetParameterCount())
		RW_THROW("No parameters enabled.");

	return doStep(step);
}

Calibration::Calibration() :
		_isEnabled(true), _isApplied(false) {
}

}
}
