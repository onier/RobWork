/*
 * SerialDeviceCalibration.cpp
 *
 *  Created on: Jul 4, 2012
 *      Author: bing
 */

#include "SerialDeviceCalibration.hpp"

#include "Pose6D.hpp"
#include <rw/models.hpp>
#include <QtCore>

namespace rwlibs {
namespace calibration {

using namespace rwlibs::calibration;

SerialDeviceCalibration::SerialDeviceCalibration(rw::models::SerialDevice::Ptr serialDevice) :
		_serialDevice(serialDevice) {
	_baseCalibration = rw::common::ownedPtr(
			new FixedFrameCalibration(rw::kinematics::Frame::Ptr(_serialDevice->getBase()).cast<rw::kinematics::FixedFrame>(), false));
	_endCalibration = rw::common::ownedPtr(
			new FixedFrameCalibration(rw::kinematics::Frame::Ptr(_serialDevice->getEnd()).cast<rw::kinematics::FixedFrame>(), true));

	_compositeDHParameterCalibration = rw::common::ownedPtr(new CompositeCalibration<DHParameterCalibration>());
//	_compositeEncoderParameterCalibration = rw::common::ownedPtr(new CompositeCalibration<EncoderParameterCalibration>());
	std::vector<rw::models::Joint*> joints(_serialDevice->getJoints());
	for (std::vector<rw::models::Joint*>::iterator jointIterator = joints.begin(); jointIterator != joints.end(); jointIterator++) {
		// Add only DH parameter calibrations for intermediate links.
		if (jointIterator != joints.begin()) {
			DHParameterCalibration::Ptr calibration = rw::common::ownedPtr(new DHParameterCalibration(*jointIterator));
			// Lock d and theta for first link.
			if (_compositeDHParameterCalibration->getCalibrations().size() == 0)
				calibration->setLockedParameters(false, true, false, true);
			_compositeDHParameterCalibration->add(calibration);
		}
//		_compositeEncoderParameterCalibration->add(rw::common::ownedPtr(new EncoderParameterCalibration(serialDevice, *jointIterator)));
	}

	add(_baseCalibration.cast<Calibration>());
	add(_endCalibration.cast<Calibration>());
	add(_compositeDHParameterCalibration.cast<Calibration>());
//	add(_compositeEncoderParameterCalibration.cast<Calibration>());
}

SerialDeviceCalibration::SerialDeviceCalibration(rw::models::SerialDevice::Ptr serialDevice, FixedFrameCalibration::Ptr baseCalibration,
		FixedFrameCalibration::Ptr endCalibration, const CompositeCalibration<DHParameterCalibration>::Ptr& compositeDHParameterCalibration/*,
		 const CompositeCalibration<EncoderParameterCalibration>::Ptr& compositeEncoderParameterCalibration*/) :
		_serialDevice(serialDevice), _baseCalibration(baseCalibration), _endCalibration(endCalibration), _compositeDHParameterCalibration(
				compositeDHParameterCalibration) /*, _compositeEncoderParameterCalibration(compositeEncoderParameterCalibration)*/{
	add(_baseCalibration.cast<Calibration>());
	add(_endCalibration.cast<Calibration>());
	add(_compositeDHParameterCalibration.cast<Calibration>());
//	add(_compositeEncoderParameterCalibration.cast<Calibration>());
}

SerialDeviceCalibration::~SerialDeviceCalibration() {

}

rw::models::SerialDevice::Ptr SerialDeviceCalibration::getDevice() const {
	return _serialDevice;
}

FixedFrameCalibration::Ptr SerialDeviceCalibration::getBaseCalibration() const {
	return _baseCalibration;
}

FixedFrameCalibration::Ptr SerialDeviceCalibration::getEndCalibration() const {
	return _endCalibration;
}

CompositeCalibration<DHParameterCalibration>::Ptr SerialDeviceCalibration::getCompositeDHParameterCalibration() const {
	return _compositeDHParameterCalibration;
}

//CompositeCalibration<EncoderParameterCalibration>::Ptr SerialDeviceCalibration::getCompositeEncoderParameterCalibration() const {
//	return _compositeEncoderParameterCalibration;
//}

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
		if (calibration->isApplied() && !calibration->isLocked())
			calibration->revert();
		propertyMap.erase("Calibration");
	}
	propertyMap.add<SerialDeviceCalibration::Ptr>("Calibration", "Calibration of serial device", calibration);
}

}
}
