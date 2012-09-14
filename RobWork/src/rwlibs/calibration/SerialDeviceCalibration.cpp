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
			new FixedFrameCalibration(rw::kinematics::Frame::Ptr(_serialDevice->getBase()).cast<rw::kinematics::FixedFrame>(), true));
	_endCalibration = rw::common::ownedPtr(
			new FixedFrameCalibration(rw::kinematics::Frame::Ptr(_serialDevice->getEnd()).cast<rw::kinematics::FixedFrame>(), false));

	_compositeDHParameterCalibration = rw::common::ownedPtr(new CompositeCalibration<DHParameterCalibration>());
	_compositeEncoderParameterCalibration = rw::common::ownedPtr(new CompositeCalibration<EncoderParameterCalibration>());
	std::vector<rw::models::Joint*> joints(_serialDevice->getJoints());
	for (std::vector<rw::models::Joint*>::iterator jointIterator = joints.begin(); jointIterator != joints.end(); jointIterator++) {
		// Add only DH parameter calibrations for intermediate links.
		if (jointIterator != joints.begin()) {
			DHParameterCalibration::Ptr calibration = rw::common::ownedPtr(new DHParameterCalibration(*jointIterator));
			// Enable only a and alpha for first link.
			if (_compositeDHParameterCalibration->getCalibrations().size() == 0)
				calibration->setEnabledParameters(true, false, true, false);
			_compositeDHParameterCalibration->add(calibration);
		}
		_compositeEncoderParameterCalibration->add(rw::common::ownedPtr(new EncoderParameterCalibration(serialDevice, *jointIterator)));
	}

	add(_baseCalibration.cast<Calibration>());
	add(_endCalibration.cast<Calibration>());
	add(_compositeDHParameterCalibration.cast<Calibration>());
	add(_compositeEncoderParameterCalibration.cast<Calibration>());
}

SerialDeviceCalibration::SerialDeviceCalibration(rw::models::SerialDevice::Ptr serialDevice, FixedFrameCalibration::Ptr baseCalibration,
		FixedFrameCalibration::Ptr endCalibration, const CompositeCalibration<DHParameterCalibration>::Ptr& compositeDHParameterCalibration,
		const CompositeCalibration<EncoderParameterCalibration>::Ptr& compositeEncoderParameterCalibration) :
		_serialDevice(serialDevice), _baseCalibration(baseCalibration), _endCalibration(endCalibration), _compositeDHParameterCalibration(
				compositeDHParameterCalibration), _compositeEncoderParameterCalibration(compositeEncoderParameterCalibration) {
	add(_baseCalibration.cast<Calibration>());
	add(_endCalibration.cast<Calibration>());
	add(_compositeDHParameterCalibration.cast<Calibration>());
	add(_compositeEncoderParameterCalibration.cast<Calibration>());
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

const CompositeCalibration<DHParameterCalibration>::Ptr& SerialDeviceCalibration::getCompositeDHParameterCalibration() const {
	return _compositeDHParameterCalibration;
}

const CompositeCalibration<EncoderParameterCalibration>::Ptr& SerialDeviceCalibration::getCompositeEncoderParameterCalibration() const {
	return _compositeEncoderParameterCalibration;
}

void SerialDeviceCalibration::save(std::string fileName) {
	QFile file(QString::fromStdString(fileName));
	file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate);

	QDomDocument document("SerialDeviceCalibration");

	QDomElement elmRoot = document.createElement("SerialDeviceCalibration");

	// Save base correction
	if (!_baseCalibration.isNull()) {
		QDomElement elmBase = document.createElement("BaseFrameCalibration");
		elmBase.appendChild(_baseCalibration->toXml(document));
		elmRoot.appendChild(elmBase);
	}

	// Save end correction
	if (!_endCalibration.isNull()) {
		QDomElement elmEnd = document.createElement("EndFrameCalibration");
		elmEnd.appendChild(_endCalibration->toXml(document));
		elmRoot.appendChild(elmEnd);
	}

	// Save dh corrections
	std::vector<DHParameterCalibration::Ptr> dhParameterCalibrations = _compositeDHParameterCalibration->getCalibrations();
	if (dhParameterCalibrations.size() > 0) {
		QDomElement dhCorrections = document.createElement("DHParameterCalibrations");
		for (std::vector<DHParameterCalibration::Ptr>::iterator it = dhParameterCalibrations.begin(); it != dhParameterCalibrations.end(); ++it) {
			DHParameterCalibration::Ptr dhParameterCalibration = (*it);
			dhCorrections.appendChild(dhParameterCalibration->toXml(document));
		}
		elmRoot.appendChild(dhCorrections);
	}

	// Save encoder corrections
	std::vector<EncoderParameterCalibration::Ptr> encoderParameterCalibrations = _compositeEncoderParameterCalibration->getCalibrations();
	if (encoderParameterCalibrations.size() > 0) {
		QDomElement encoderCorrections = document.createElement("EncoderParameterCalibrations");
		for (std::vector<EncoderParameterCalibration::Ptr>::iterator it = encoderParameterCalibrations.begin();
				it != encoderParameterCalibrations.end(); ++it) {
			EncoderParameterCalibration::Ptr encoderCalibration = (*it);
			encoderCorrections.appendChild(encoderCalibration->toXml(document));
		}
		elmRoot.appendChild(encoderCorrections);
	}

	document.appendChild(elmRoot);

	QTextStream textStream(&file);
	textStream.setRealNumberPrecision(16);
	textStream << document.toString();

	file.close();
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
	propertyMap.add<SerialDeviceCalibration::Ptr>("Calibration", "Calibration of serial device", calibration);
}

SerialDeviceCalibration::Ptr SerialDeviceCalibration::load(rw::kinematics::StateStructure::Ptr stateStructure, rw::models::SerialDevice::Ptr device,
		std::string filePath) {
	QFile file(QString::fromStdString(filePath));
	file.open(QIODevice::ReadOnly | QIODevice::Text | QIODevice::Truncate);

	QDomDocument document("SerialDeviceCalibration");

	if (!document.setContent(&file))
		RW_THROW("Content not set.");

	QDomElement elmRoot = document.documentElement();
	if (elmRoot.tagName() != "SerialDeviceCalibration")
		RW_THROW("Element not found.");

	// Load base frame calibration
	FixedFrameCalibration::Ptr baseCalibration;
	QDomNode nodeBase = elmRoot.namedItem("BaseFrameCalibration");
	if (!nodeBase.isNull() && nodeBase.hasChildNodes())
		baseCalibration = FixedFrameCalibration::fromXml(nodeBase.childNodes().at(0).toElement(), stateStructure);

	// Load end frame calibration
	FixedFrameCalibration::Ptr endCalibration;
	QDomNode nodeEnd = elmRoot.namedItem("EndFrameCalibration");
	if (!nodeEnd.isNull() && nodeEnd.hasChildNodes())
		endCalibration = FixedFrameCalibration::fromXml(nodeEnd.childNodes().at(0).toElement(), stateStructure);

	// Load DH calibrations
	CompositeCalibration<DHParameterCalibration>::Ptr dhCalibrations = rw::common::ownedPtr(new CompositeCalibration<DHParameterCalibration>());
	QDomNode nodeDH = elmRoot.namedItem("DHParameterCalibrations");
	if (!nodeDH.isNull()) {
		QDomNodeList nodes = nodeDH.childNodes();
		for (int nodeNo = 0; nodeNo < nodes.size(); nodeNo++) {
			QDomElement element = nodes.at(nodeNo).toElement();
			dhCalibrations->add(DHParameterCalibration::fromXml(element, stateStructure));
		}
	}

	// Load encoder calibrations
	CompositeCalibration<EncoderParameterCalibration>::Ptr encoderCalibrations = rw::common::ownedPtr(new CompositeCalibration<EncoderParameterCalibration>());
	QDomNode nodeEncoder = elmRoot.namedItem("JointEncoderCalibrations");
	if (!nodeEncoder.isNull()) {
		QDomNodeList nodes = nodeEncoder.childNodes();
		for (int nodeNo = 0; nodeNo < nodes.size(); nodeNo++) {
			QDomElement element = nodes.at(nodeNo).toElement();
			encoderCalibrations->add(EncoderParameterCalibration::fromXml(element, stateStructure, device));
		}
	}

	SerialDeviceCalibration::Ptr calibration = rw::common::ownedPtr(
			new SerialDeviceCalibration(device, baseCalibration, endCalibration, dhCalibrations, encoderCalibrations));

	return calibration;
}

}
}
