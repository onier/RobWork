/*
 * SerialDeviceCalibration.cpp
 *
 *  Created on: Jul 4, 2012
 *      Author: bing
 */

#include "SerialDeviceCalibration.hpp"

#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

using namespace rwlibs::calibration;

SerialDeviceCalibration::SerialDeviceCalibration(rw::models::SerialDevice::Ptr device) :
		_device(device) {
	_baseCalibration = rw::common::ownedPtr(
			new FixedFrameCalibration(rw::kinematics::Frame::Ptr(device->getBase()).cast<rw::kinematics::FixedFrame>(), true));

	_endCalibration = rw::common::ownedPtr(
		new FixedFrameCalibration(rw::kinematics::Frame::Ptr(device->getEnd()).cast<rw::kinematics::FixedFrame>(), false));
	
	_internalLinkCalibration = rw::common::ownedPtr(new CompositeCalibration<DHParameterCalibration>());
	std::vector<rw::models::Joint*> joints = device->getJoints();
	for (std::vector<rw::models::Joint*>::iterator jointIterator = joints.begin(); jointIterator != joints.end(); jointIterator++) {
		// Add DH parameter calibrations for intermediate links.
		if (jointIterator != joints.begin()) {
			rw::models::Joint::Ptr joint = (*jointIterator);
			DHParameterCalibration::Ptr calibration = rw::common::ownedPtr(new DHParameterCalibration(joint));
			_internalLinkCalibration->addCalibration(calibration);

			// Lock d and theta for first link.
			if (jointIterator == ++(joints.begin())) {
				calibration->setParameterEnabled(DHParameterCalibration::PARAMETER_D, false);
				calibration->setParameterEnabled(DHParameterCalibration::PARAMETER_THETA, false);
			}
		}
	}
	
	addCalibration(_baseCalibration.cast<Calibration>());
	addCalibration(_endCalibration.cast<Calibration>());
	addCalibration(_internalLinkCalibration.cast<Calibration>());
}

SerialDeviceCalibration::SerialDeviceCalibration(rw::models::SerialDevice::Ptr device, FixedFrameCalibration::Ptr baseCalibration, FixedFrameCalibration::Ptr endCalibration,
			const CompositeCalibration<DHParameterCalibration>::Ptr& internalLinkCalibration) :
		_device(device), _baseCalibration(baseCalibration), _endCalibration(endCalibration), _internalLinkCalibration(internalLinkCalibration) {
	addCalibration(_baseCalibration.cast<Calibration>());
	addCalibration(_endCalibration.cast<Calibration>());
	addCalibration(_internalLinkCalibration.cast<Calibration>());
}

SerialDeviceCalibration::~SerialDeviceCalibration() {

}

rw::models::SerialDevice::Ptr SerialDeviceCalibration::getDevice() const {
	return _device;
}

FixedFrameCalibration::Ptr SerialDeviceCalibration::getBaseCalibration() const {
	return _baseCalibration;
}

FixedFrameCalibration::Ptr SerialDeviceCalibration::getEndCalibration() const {
	return _endCalibration;
}

CompositeCalibration<DHParameterCalibration>::Ptr SerialDeviceCalibration::getInternalLinkCalibration() const {
	return _internalLinkCalibration;
}

SerialDeviceCalibration::Ptr SerialDeviceCalibration::get(rw::models::SerialDevice::Ptr device) {
	return get(device->getPropertyMap());
}

SerialDeviceCalibration::Ptr SerialDeviceCalibration::get(const rw::common::PropertyMap& propertyMap) {
	SerialDeviceCalibration::Ptr calibration;
	if (propertyMap.has("Calibration"))
		calibration = propertyMap.get<SerialDeviceCalibration::Ptr>("Calibration");
	return calibration;
}

void SerialDeviceCalibration::set(SerialDeviceCalibration::Ptr calibration, rw::models::SerialDevice::Ptr device) {
	set(calibration, device->getPropertyMap());
}

void SerialDeviceCalibration::set(SerialDeviceCalibration::Ptr calibration, rw::common::PropertyMap& propertyMap) {
	if (propertyMap.has("Calibration")) {
		if (calibration->isApplied())
			calibration->revert();
		propertyMap.erase("Calibration");
	}
	propertyMap.add<SerialDeviceCalibration::Ptr>("Calibration", "Calibration of SerialDevice", calibration);
}

}
}
