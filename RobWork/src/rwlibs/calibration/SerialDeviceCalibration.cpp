/*
 * SerialDeviceCalibration.cpp
 *
 *  Created on: Jul 4, 2012
 *      Author: bing
 */

#include "SerialDeviceCalibration.hpp"

#include "Pose6D.hpp"
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

using namespace rwlibs::calibration;

SerialDeviceCalibration::SerialDeviceCalibration(rw::models::SerialDevice::Ptr serialDevice) :
		_isEnabled(true), _isApplied(false), _serialDevice(serialDevice) {
	_baseCalibration = rw::common::ownedPtr(
			new FixedFrameCalibration(rw::kinematics::Frame::Ptr(_serialDevice->getBase()).cast<rw::kinematics::FixedFrame>(), true));
	_endCalibration = rw::common::ownedPtr(
			new FixedFrameCalibration(rw::kinematics::Frame::Ptr(_serialDevice->getEnd()).cast<rw::kinematics::FixedFrame>(), false));
	std::vector<rw::models::Joint*> joints(_serialDevice->getJoints());
	for (std::vector<rw::models::Joint*>::iterator jointIterator = joints.begin(); jointIterator != joints.end(); jointIterator++) {
		// Add only DH parameter calibrations for intermediate links.
		if (jointIterator != joints.begin())
			_dhParameterCalibrations.append(rw::common::ownedPtr(new DHParameterCalibration(*jointIterator)));
		_encoderDecentralizationCalibrations.append(rw::common::ownedPtr(new EncoderParameterCalibration(serialDevice, *jointIterator)));
	}

	_calibrations.append(_baseCalibration.cast<PoseCalibration>());
	_calibrations.append(_endCalibration.cast<PoseCalibration>());
	QListIterator<DHParameterCalibration::Ptr> dhParameterCalibrationIterator(_dhParameterCalibrations);
	while (dhParameterCalibrationIterator.hasNext())
		_calibrations.append(dhParameterCalibrationIterator.next().cast<PoseCalibration>());
	QListIterator<EncoderParameterCalibration::Ptr> encoderDecentralizationCalibrationIterator(_encoderDecentralizationCalibrations);
	while (encoderDecentralizationCalibrationIterator.hasNext())
		_calibrations.append(encoderDecentralizationCalibrationIterator.next().cast<PoseCalibration>());
}

SerialDeviceCalibration::SerialDeviceCalibration(rw::models::SerialDevice::Ptr serialDevice, FixedFrameCalibration::Ptr baseCalibration,
		FixedFrameCalibration::Ptr endCalibration, const QList<DHParameterCalibration::Ptr>& dhParameterCalibrations,
		const QList<EncoderParameterCalibration::Ptr>& encoderDecentralizationCalibrations) :
		_isEnabled(true), _isApplied(false), _serialDevice(serialDevice), _baseCalibration(baseCalibration), _endCalibration(endCalibration), _dhParameterCalibrations(
				dhParameterCalibrations), _encoderDecentralizationCalibrations(encoderDecentralizationCalibrations) {
	_calibrations.append(_baseCalibration.cast<PoseCalibration>());
	_calibrations.append(_endCalibration.cast<PoseCalibration>());
	QListIterator<DHParameterCalibration::Ptr> dhParameterCalibrationIterator(_dhParameterCalibrations);
	while (dhParameterCalibrationIterator.hasNext())
		_calibrations.append(dhParameterCalibrationIterator.next().cast<PoseCalibration>());
	QListIterator<EncoderParameterCalibration::Ptr> encoderDecentralizationCalibrationIterator(_encoderDecentralizationCalibrations);
	while (encoderDecentralizationCalibrationIterator.hasNext())
		_calibrations.append(encoderDecentralizationCalibrationIterator.next().cast<PoseCalibration>());
}

SerialDeviceCalibration::~SerialDeviceCalibration() {

}

bool SerialDeviceCalibration::isEnabled() const {
	return _isEnabled;
}

void SerialDeviceCalibration::apply() {
	if (!_isEnabled)
		RW_THROW("Not enabled.");
	if (_isApplied)
		RW_WARN("Already applied.");

	QListIterator<PoseCalibration::Ptr> calibrationIterator(_calibrations);
	while (calibrationIterator.hasNext()) {
		PoseCalibration::Ptr calibration = calibrationIterator.next();
		if (calibration->isEnabled())
			calibration->apply();
	}

	_isApplied = true;
}

void SerialDeviceCalibration::revert() {
	if (!_isEnabled)
		RW_THROW("Not enabled.");
	if (!_isApplied)
		RW_WARN("Not applied.");

	QListIterator<PoseCalibration::Ptr> calibrationIterator(_calibrations);
	while (calibrationIterator.hasNext()) {
		PoseCalibration::Ptr calibration = calibrationIterator.next();
		if (calibration->isEnabled())
			calibration->revert();
	}

	_isApplied = false;
}

void SerialDeviceCalibration::correct(rw::kinematics::State& state) {
	if (!_isEnabled)
		RW_THROW("Not enabled.");
	if (!_isApplied)
		RW_WARN("Not applied.");

	QListIterator<PoseCalibration::Ptr> calibrationIterator(_calibrations);
	while (calibrationIterator.hasNext()) {
		PoseCalibration::Ptr calibration = calibrationIterator.next();
		if (calibration->isEnabled())
			calibration->correct(state);
	}
}

bool SerialDeviceCalibration::isApplied() const {
	return _isApplied;
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

QList<DHParameterCalibration::Ptr> SerialDeviceCalibration::getDHParameterCalibrations() const {
	return _dhParameterCalibrations;
}

QList<EncoderParameterCalibration::Ptr> SerialDeviceCalibration::getEncoderDecentralizationCalibrations() const {
	return _encoderDecentralizationCalibrations;
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
		QListIterator<DHParameterCalibration::Ptr> it(_dhParameterCalibrations);
		while (it.hasNext()) {
			DHParameterCalibration::Ptr dhParameterCalibration = it.next();
			dhCorrections.appendChild(dhParameterCalibration->toXml(document));
		}
		elmRoot.appendChild(dhCorrections);
	}

	// Save encoder corrections
	if (_encoderDecentralizationCalibrations.size() > 0) {
		QDomElement encoderCorrections = document.createElement("JointEncoderCalibrations");
		QListIterator<EncoderParameterCalibration::Ptr> it(_encoderDecentralizationCalibrations);
		while (it.hasNext()) {
			EncoderParameterCalibration::Ptr encoderCalibration = it.next();
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

SerialDeviceCalibration::Ptr SerialDeviceCalibration::load(rw::kinematics::StateStructure::Ptr stateStructure, rw::models::SerialDevice::Ptr device, std::string fileName) {
	QFile file(QString::fromStdString(fileName));
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
	QList<DHParameterCalibration::Ptr> dhCalibrations;
	QDomNode nodeDH = elmRoot.namedItem("DHParameterCalibrations");
	if (!nodeDH.isNull()) {
		QDomNodeList nodes = nodeDH.childNodes();
		for (int nodeNo = 0; nodeNo < nodes.size(); nodeNo++) {
			QDomElement element = nodes.at(nodeNo).toElement();
			dhCalibrations.append(DHParameterCalibration::fromXml(element, stateStructure));
		}
	}

	// Load encoder calibrations
	QList<EncoderParameterCalibration::Ptr> encoderCalibrations;
	QDomNode nodeEncoder = elmRoot.namedItem("JointEncoderCalibrations");
	if (!nodeEncoder.isNull()) {
		QDomNodeList nodes = nodeEncoder.childNodes();
		for (int nodeNo = 0; nodeNo < nodes.size(); nodeNo++) {
			QDomElement element = nodes.at(nodeNo).toElement();
			encoderCalibrations.append(EncoderParameterCalibration::fromXml(element, stateStructure, device));
		}
	}

	SerialDeviceCalibration::Ptr serialDeviceCalibration = rw::common::ownedPtr(
			new SerialDeviceCalibration(device, baseCalibration, endCalibration, dhCalibrations, encoderCalibrations));

	set(serialDeviceCalibration, device);

	return serialDeviceCalibration;
}

SerialDeviceCalibration::Ptr SerialDeviceCalibration::get(rw::models::SerialDevice::Ptr serialDevice) {
	SerialDeviceCalibration::Ptr serialDeviceCalibration;
	rw::common::PropertyMap propertyMap = serialDevice->getPropertyMap();
	if (propertyMap.has("Calibration"))
		serialDeviceCalibration = propertyMap.get<SerialDeviceCalibration::Ptr>("Calibration");
	return serialDeviceCalibration;
}

void SerialDeviceCalibration::set(SerialDeviceCalibration::Ptr serialDeviceCalibration, rw::models::SerialDevice::Ptr serialDevice) {
	rw::common::PropertyMap propertyMap = serialDevice->getPropertyMap();
	if (propertyMap.has("Calibration")) {
		if (serialDeviceCalibration->isApplied())
			serialDeviceCalibration->revert();
		propertyMap.erase("Calibration");
	}
	propertyMap.add<SerialDeviceCalibration::Ptr>("Calibration", "", serialDeviceCalibration);
}

}
}
