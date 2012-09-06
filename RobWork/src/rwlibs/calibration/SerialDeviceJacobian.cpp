/*
 * SerialDeviceJacobian.cpp
 *
 *  Created on: Aug 30, 2012
 *      Author: bing
 */

#include "SerialDeviceJacobian.hpp"

namespace rwlibs {
namespace calibration {

SerialDeviceJacobian::SerialDeviceJacobian(SerialDeviceCalibration::Ptr calibration) : _calibration(calibration) {
	_baseJacobian = rw::common::ownedPtr(new FixedFrameJacobian(calibration->getBaseCalibration()));
	_endJacobian = rw::common::ownedPtr(new FixedFrameJacobian(calibration->getEndCalibration()));
	QListIterator<DHParameterCalibration::Ptr> dhParameterCalibrationIterator(calibration->getDHParameterCalibrations());
	while (dhParameterCalibrationIterator.hasNext()) {
		bool isFirst = !dhParameterCalibrationIterator.hasPrevious();
		// Enable only a and alpha for first link.
		DHParameterJacobian::Ptr dhParameterJacobian = rw::common::ownedPtr(new DHParameterJacobian(dhParameterCalibrationIterator.next()));
		if (isFirst)
			dhParameterJacobian->setEnabledParameters(true, false, true, false);
		_dhParameterJacobians.append(dhParameterJacobian);
	}
	QListIterator<EncoderParameterCalibration::Ptr> encoderDecentralizationCalibrationIterator(calibration->getEncoderDecentralizationCalibrations());
	while (encoderDecentralizationCalibrationIterator.hasNext())
		_encoderDecentralizationJacobians.append(rw::common::ownedPtr(new EncoderParameterJacobian(calibration->getDevice(), encoderDecentralizationCalibrationIterator.next())));

	_jacobians.append(_baseJacobian.cast<DeviceJacobian>());
	_jacobians.append(_endJacobian.cast<DeviceJacobian>());
	QListIterator<DHParameterJacobian::Ptr> dhParameterJacobianIterator(_dhParameterJacobians);
	while (dhParameterJacobianIterator.hasNext())
		_jacobians.append(dhParameterJacobianIterator.next().cast<DeviceJacobian>());
	QListIterator<EncoderParameterJacobian::Ptr> encoderDecentralizationJacobianIterator(_encoderDecentralizationJacobians);
	while (encoderDecentralizationJacobianIterator.hasNext())
		_jacobians.append(encoderDecentralizationJacobianIterator.next().cast<DeviceJacobian>());
}

SerialDeviceJacobian::~SerialDeviceJacobian() {

}

DeviceCalibration::Ptr SerialDeviceJacobian::getCalibration() const {
	return _calibration;
}

int SerialDeviceJacobian::getParameterCount() const {
	int parameterCount = 0;
	QListIterator<DeviceJacobian::Ptr> jacobianIterator(_jacobians);
	while (jacobianIterator.hasNext())
		parameterCount += jacobianIterator.next()->getParameterCount();
	return parameterCount;
}

Eigen::MatrixXd SerialDeviceJacobian::compute(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
		const rw::kinematics::State& state) {
	// FIXME: "- 1" hack?!
	unsigned int parameterNo = -1;
	QListIterator<DeviceJacobian::Ptr> jacobianIterator(_jacobians);
	Eigen::MatrixXd jacobian(6, getParameterCount());
	while (jacobianIterator.hasNext()) {
		DeviceJacobian::Ptr poseJacobian = jacobianIterator.next();
		unsigned int parameterCount = poseJacobian->getParameterCount();
		if (parameterCount > 0) {
			jacobian.block(6, parameterNo, 6, parameterCount) = poseJacobian->compute(referenceFrame, measurementFrame, state);
			parameterNo += parameterCount;
		}
	}
	return jacobian;
}

void SerialDeviceJacobian::step(const Eigen::VectorXd& step) {
	unsigned int parameterNo = 0;
	QListIterator<DeviceJacobian::Ptr> jacobianIterator(_jacobians);
	while (jacobianIterator.hasNext()) {
		DeviceJacobian::Ptr jacobian = jacobianIterator.next();
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

QList<DHParameterJacobian::Ptr> SerialDeviceJacobian::getDHParameterJacobians() const {
	return _dhParameterJacobians;
}

void SerialDeviceJacobian::setDHParameterJacobiansEnabled(bool isEnabled) {
	QListIterator<DHParameterJacobian::Ptr> jacobianIterator(_dhParameterJacobians);
	while (jacobianIterator.hasNext())
		jacobianIterator.next()->setEnabled(isEnabled);
}

QList<EncoderParameterJacobian::Ptr> SerialDeviceJacobian::getEncoderDecentralizationJacobians() const {
	return _encoderDecentralizationJacobians;
}

void SerialDeviceJacobian::setEncoderDecentralizationJacobiansEnabled(bool isEnabled) {
	QListIterator<EncoderParameterJacobian::Ptr> jacobianIterator(_encoderDecentralizationJacobians);
	while (jacobianIterator.hasNext())
		jacobianIterator.next()->setEnabled(isEnabled);
}

}
}
