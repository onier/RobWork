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
	std::vector<rw::models::Joint*> joints(_serialDevice->getJoints());
	for (std::vector<rw::models::Joint*>::iterator jointIterator = joints.begin(); jointIterator != joints.end(); jointIterator++) {
		// Add only DH parameter calibrations for intermediate links.
		if (jointIterator != joints.begin())
			_dhParameterCalibrations.push_back(rw::common::ownedPtr(new DHParameterCalibration(*jointIterator)));
		_encoderParameterCalibrations.push_back(rw::common::ownedPtr(new EncoderParameterCalibration(serialDevice, *jointIterator)));
	}

	_calibrations.push_back(_baseCalibration.cast<Calibration>());
	_calibrations.push_back(_endCalibration.cast<Calibration>());
	for (std::vector<DHParameterCalibration::Ptr>::iterator it = _dhParameterCalibrations.begin(); it != _dhParameterCalibrations.end(); ++it)
		_calibrations.push_back((*it).cast<Calibration>());
	for (std::vector<EncoderParameterCalibration::Ptr>::iterator it = _encoderParameterCalibrations.begin(); it != _encoderParameterCalibrations.end(); ++it)
		_calibrations.push_back((*it).cast<Calibration>());
}

SerialDeviceCalibration::SerialDeviceCalibration(rw::models::SerialDevice::Ptr serialDevice, FixedFrameCalibration::Ptr baseCalibration,
		FixedFrameCalibration::Ptr endCalibration, const std::vector<DHParameterCalibration::Ptr>& dhParameterCalibrations,
		const std::vector<EncoderParameterCalibration::Ptr>& encoderDecentralizationCalibrations) :
		_serialDevice(serialDevice), _baseCalibration(baseCalibration), _endCalibration(endCalibration), _dhParameterCalibrations(dhParameterCalibrations), _encoderParameterCalibrations(
				encoderDecentralizationCalibrations) {
	_calibrations.push_back(_baseCalibration.cast<Calibration>());
	_calibrations.push_back(_endCalibration.cast<Calibration>());
	for (std::vector<DHParameterCalibration::Ptr>::iterator it = _dhParameterCalibrations.begin(); it != _dhParameterCalibrations.end(); ++it)
		_calibrations.push_back((*it).cast<Calibration>());
	for (std::vector<EncoderParameterCalibration::Ptr>::iterator it = _encoderParameterCalibrations.begin(); it != _encoderParameterCalibrations.end(); ++it)
		_calibrations.push_back((*it).cast<Calibration>());
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

std::vector<DHParameterCalibration::Ptr> SerialDeviceCalibration::getDHParameterCalibrations() const {
	return _dhParameterCalibrations;
}

std::vector<EncoderParameterCalibration::Ptr> SerialDeviceCalibration::getEncoderParameterCalibrations() const {
	return _encoderParameterCalibrations;
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
	if (_dhParameterCalibrations.size() > 0) {
		QDomElement dhCorrections = document.createElement("DHParameterCalibrations");
		for (std::vector<DHParameterCalibration::Ptr>::iterator it = _dhParameterCalibrations.begin(); it != _dhParameterCalibrations.end(); ++it) {
			DHParameterCalibration::Ptr dhParameterCalibration = (*it);
			dhCorrections.appendChild(dhParameterCalibration->toXml(document));
		}
		elmRoot.appendChild(dhCorrections);
	}

	// Save encoder corrections
	if (_encoderParameterCalibrations.size() > 0) {
		QDomElement encoderCorrections = document.createElement("EncoderParameterCalibrations");
		for (std::vector<EncoderParameterCalibration::Ptr>::iterator it = _encoderParameterCalibrations.begin(); it != _encoderParameterCalibrations.end(); ++it) {
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
	std::vector<DHParameterCalibration::Ptr> dhCalibrations;
	QDomNode nodeDH = elmRoot.namedItem("DHParameterCalibrations");
	if (!nodeDH.isNull()) {
		QDomNodeList nodes = nodeDH.childNodes();
		for (int nodeNo = 0; nodeNo < nodes.size(); nodeNo++) {
			QDomElement element = nodes.at(nodeNo).toElement();
			dhCalibrations.push_back(DHParameterCalibration::fromXml(element, stateStructure));
		}
	}

	// Load encoder calibrations
	std::vector<EncoderParameterCalibration::Ptr> encoderCalibrations;
	QDomNode nodeEncoder = elmRoot.namedItem("JointEncoderCalibrations");
	if (!nodeEncoder.isNull()) {
		QDomNodeList nodes = nodeEncoder.childNodes();
		for (int nodeNo = 0; nodeNo < nodes.size(); nodeNo++) {
			QDomElement element = nodes.at(nodeNo).toElement();
			encoderCalibrations.push_back(EncoderParameterCalibration::fromXml(element, stateStructure, device));
		}
	}

	SerialDeviceCalibration::Ptr calibration = rw::common::ownedPtr(
			new SerialDeviceCalibration(device, baseCalibration, endCalibration, dhCalibrations, encoderCalibrations));

	return calibration;
}

void SerialDeviceCalibration::doApply() {
	for (std::vector<Calibration::Ptr>::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		Calibration::Ptr calibration = (*it);
		if (calibration->isEnabled())
			calibration->apply();
	}
}

void SerialDeviceCalibration::doRevert() {
	for (std::vector<Calibration::Ptr>::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		Calibration::Ptr calibration = (*it);
		if (calibration->isEnabled())
			calibration->revert();
	}
}

void SerialDeviceCalibration::doCorrect(rw::kinematics::State& state) {
	for (std::vector<Calibration::Ptr>::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		Calibration::Ptr calibration = (*it);
		if (calibration->isEnabled())
			calibration->correct(state);
	}
}

}
}
