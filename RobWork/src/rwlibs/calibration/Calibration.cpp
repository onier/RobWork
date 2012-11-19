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
	_isLocked = isLocked;
}

bool Calibration::isApplied() const {
	return _isApplied;
}

void Calibration::apply() {
	RW_ASSERT(!isLocked());
	RW_ASSERT(!isApplied());

	doApply();

	_isApplied = true;
}

void Calibration::revert() {
	RW_ASSERT(!isLocked());
	RW_ASSERT(isApplied());

	doRevert();

	_isApplied = false;
}

void Calibration::correct(rw::kinematics::State& state) {
	RW_ASSERT(isApplied());

	doCorrect(state);
}

int Calibration::getParameterCount() const {
	return _isLocked ? 0 : doGetParameterCount();
}

Eigen::MatrixXd Calibration::computeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame, const rw::kinematics::State& state) {
	RW_ASSERT(getParameterCount() != 0);

	return doComputeJacobian(referenceFrame, targetFrame, state);
}

void Calibration::takeStep(const Eigen::VectorXd& step) {
	RW_ASSERT(getParameterCount() != 0);

	return doTakeStep(step);
}

Calibration::Calibration() :
		_isLocked(false), _isApplied(false) {
}

}
}
