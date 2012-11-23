/*
 * SerialDeviceJacobian.cpp
 *
 *  Created on: Nov 20, 2012
 *      Author: bing
 */

#include "SerialDeviceJacobian.hpp"

#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

using namespace rwlibs::calibration;

SerialDeviceJacobian::SerialDeviceJacobian(SerialDeviceCalibration::Ptr calibration) {
	_baseJacobian = rw::common::ownedPtr(new FixedFrameJacobian(calibration->getBaseCalibration()));
	_endJacobian = rw::common::ownedPtr(new FixedFrameJacobian(calibration->getEndCalibration()));
	_compositeDHParameterJacobian = rw::common::ownedPtr(new CompositeJacobian<DHParameterJacobian>());
	std::vector<DHParameterCalibration::Ptr> dhParameterCalibrations(calibration->getCompositeDHParameterCalibration()->getCalibrations());
	for (std::vector<DHParameterCalibration::Ptr>::iterator dhParameterCalibrationIterator = dhParameterCalibrations.begin(); dhParameterCalibrationIterator != dhParameterCalibrations.end(); dhParameterCalibrationIterator++) {
		DHParameterCalibration::Ptr calibration = (*dhParameterCalibrationIterator);
		DHParameterJacobian::Ptr jacobian = rw::common::ownedPtr(new DHParameterJacobian(calibration));
		// Lock d and theta for first link.
		if (dhParameterCalibrationIterator == dhParameterCalibrations.begin()) {
			jacobian->setParameterEnabled(DHParameterJacobian::PARAMETER_D, false);
			jacobian->setParameterEnabled(DHParameterJacobian::PARAMETER_THETA, false);
		}
		_compositeDHParameterJacobian->add(jacobian);
	}

	add(_baseJacobian.cast<Jacobian>());
	add(_endJacobian.cast<Jacobian>());
	add(_compositeDHParameterJacobian.cast<Jacobian>());
}

SerialDeviceJacobian::~SerialDeviceJacobian() {

}

FixedFrameJacobian::Ptr SerialDeviceJacobian::getBaseJacobian() const {
	return _baseJacobian;
}

FixedFrameJacobian::Ptr SerialDeviceJacobian::getEndJacobian() const {
	return _endJacobian;
}

CompositeJacobian<DHParameterJacobian>::Ptr SerialDeviceJacobian::getCompositeDHParameterJacobian() const {
	return _compositeDHParameterJacobian;
}

}
}
