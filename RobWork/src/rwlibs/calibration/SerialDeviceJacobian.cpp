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
	_internalLinkJacobian = rw::common::ownedPtr(new CompositeJacobian<DHParameterJacobian>());
	std::vector<DHParameterCalibration::Ptr> dhParameterCalibrations(calibration->getInternalLinkCalibration()->getCalibrations());
	for (std::vector<DHParameterCalibration::Ptr>::iterator calibrationIterator = dhParameterCalibrations.begin(); calibrationIterator != dhParameterCalibrations.end(); calibrationIterator++) {
		DHParameterCalibration::Ptr calibration = (*calibrationIterator);
		DHParameterJacobian::Ptr jacobian = rw::common::ownedPtr(new DHParameterJacobian(calibration));
		_internalLinkJacobian->addJacobian(jacobian);
	}

	addJacobian(_baseJacobian.cast<Jacobian>());
	addJacobian(_endJacobian.cast<Jacobian>());
	addJacobian(_internalLinkJacobian.cast<Jacobian>());
}

SerialDeviceJacobian::~SerialDeviceJacobian() {

}

FixedFrameJacobian::Ptr SerialDeviceJacobian::getBaseJacobian() const {
	return _baseJacobian;
}

FixedFrameJacobian::Ptr SerialDeviceJacobian::getEndJacobian() const {
	return _endJacobian;
}

CompositeJacobian<DHParameterJacobian>::Ptr SerialDeviceJacobian::getInternalLinkJacobian() const {
	return _internalLinkJacobian;
}

}
}
