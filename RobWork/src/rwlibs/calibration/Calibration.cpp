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

bool Calibration::isApplied() const {
	return _isApplied;
}

void Calibration::apply() {
	RW_ASSERT(!isApplied());

	doApply();

	_isApplied = true;
}

void Calibration::revert() {
	RW_ASSERT(isApplied());

	doRevert();

	_isApplied = false;
}

void Calibration::correctState(rw::kinematics::State& state) {
	RW_ASSERT(isApplied());

	doCorrectState(state);
}

Calibration::Calibration() :
		_isApplied(false) {

}

}
}
