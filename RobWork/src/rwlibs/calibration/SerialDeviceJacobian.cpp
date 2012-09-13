/*
 * SerialDeviceJacobian.cpp
 *
 *  Created on: Aug 30, 2012
 *      Author: bing
 */

#include "SerialDeviceJacobian.hpp"

namespace rwlibs {
namespace calibration {

SerialDeviceJacobian::SerialDeviceJacobian(SerialDeviceCalibration::Ptr calibration) :
		DeviceJacobian(calibration), _calibration(calibration) {
	_baseJacobian = rw::common::ownedPtr(new FixedFrameJacobian(calibration->getBaseCalibration()));
	_endJacobian = rw::common::ownedPtr(new FixedFrameJacobian(calibration->getEndCalibration()));
	std::vector<DHParameterCalibration::Ptr> dhParameterCalibrations = calibration->getDHParameterCalibrations();
	for (std::vector<DHParameterCalibration::Ptr>::iterator it = dhParameterCalibrations.begin(); it != dhParameterCalibrations.end(); ++it) {
		bool isFirst = (it == dhParameterCalibrations.begin());
		// Enable only a and alpha for first link.
		DHParameterJacobian::Ptr dhParameterJacobian = rw::common::ownedPtr(new DHParameterJacobian((*it)));
		if (isFirst)
			dhParameterJacobian->setEnabledParameters(true, false, true, false);
		_dhParameterJacobians.push_back(dhParameterJacobian);
	}
	std::vector<EncoderParameterCalibration::Ptr> encoderParameterCalibrations = calibration->getEncoderParameterCalibrations();
	for (std::vector<EncoderParameterCalibration::Ptr>::iterator it = encoderParameterCalibrations.begin(); it != encoderParameterCalibrations.end(); ++it)
		_encoderParameterJacobians.push_back(rw::common::ownedPtr(new EncoderParameterJacobian((*it), calibration->getDevice())));

	_jacobians.push_back(_baseJacobian.cast<DeviceJacobian>());
	_jacobians.push_back(_endJacobian.cast<DeviceJacobian>());
	for (std::vector<DHParameterJacobian::Ptr>::iterator it = _dhParameterJacobians.begin(); it != _dhParameterJacobians.end(); ++it)
		_jacobians.push_back((*it).cast<DeviceJacobian>());
	for (std::vector<EncoderParameterJacobian::Ptr>::iterator it = _encoderParameterJacobians.begin(); it != _encoderParameterJacobians.end(); ++it)
		_jacobians.push_back((*it).cast<DeviceJacobian>());
}

SerialDeviceJacobian::~SerialDeviceJacobian() {

}

int SerialDeviceJacobian::getParameterCount() const {
	int parameterCount = 0;
	for (std::vector<DeviceJacobian::Ptr>::const_iterator it = _jacobians.begin(); it != _jacobians.end(); ++it)
		parameterCount += (*it)->getParameterCount();
	return parameterCount;
}

Eigen::MatrixXd SerialDeviceJacobian::compute(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
		const rw::kinematics::State& state) {
	unsigned int parameterNo = 0;
	Eigen::MatrixXd jacobian(6, getParameterCount());
	for (std::vector<DeviceJacobian::Ptr>::iterator it = _jacobians.begin(); it != _jacobians.end(); ++it) {
		DeviceJacobian::Ptr deviceJacobian = (*it);
		unsigned int parameterCount = deviceJacobian->getParameterCount();
		if (parameterCount > 0) {
			jacobian.block(0, parameterNo, 6, parameterCount) = deviceJacobian->compute(referenceFrame, measurementFrame, state);
			parameterNo += parameterCount;
		}
	}
	return jacobian;
}

void SerialDeviceJacobian::step(const Eigen::VectorXd& step) {
	unsigned int parameterNo = 0;
	for (std::vector<DeviceJacobian::Ptr>::iterator it = _jacobians.begin(); it != _jacobians.end(); ++it) {
		DeviceJacobian::Ptr jacobian = (*it);
		unsigned int parameterCount = jacobian->getParameterCount();
		if (parameterCount > 0) {
			jacobian->step(step.segment(parameterNo, parameterCount));
			parameterNo += parameterCount;
		}
	}
}

FixedFrameJacobian::Ptr SerialDeviceJacobian::getBaseJacobian() const {
	return _baseJacobian;
}

FixedFrameJacobian::Ptr SerialDeviceJacobian::getEndJacobian() const {
	return _endJacobian;
}

std::vector<DHParameterJacobian::Ptr> SerialDeviceJacobian::getDHParameterJacobians() const {
	return _dhParameterJacobians;
}

void SerialDeviceJacobian::setDHParameterJacobiansEnabled(bool isEnabled) {
	for (std::vector<DHParameterJacobian::Ptr>::iterator it = _dhParameterJacobians.begin(); it != _dhParameterJacobians.end(); ++it)
		(*it)->setEnabled(isEnabled);
}

std::vector<EncoderParameterJacobian::Ptr> SerialDeviceJacobian::getEncoderDecentralizationJacobians() const {
	return _encoderParameterJacobians;
}

void SerialDeviceJacobian::setEncoderDecentralizationJacobiansEnabled(bool isEnabled) {
	for (std::vector<EncoderParameterJacobian::Ptr>::iterator it = _encoderParameterJacobians.begin(); it != _encoderParameterJacobians.end(); ++it)
		(*it)->setEnabled(isEnabled);
}

}
}
