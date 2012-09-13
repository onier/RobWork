/*
 * SerialDeviceCalibrator.cpp
 *
 *  Created on: Feb 27, 2012
 *      Author: bing
 */

#include "SerialDeviceCalibrator.hpp"

#include <Eigen/Eigenvalues>
#include <rw/models/DHParameterSet.hpp>

namespace rwlibs {
namespace calibration {

SerialDeviceCalibrator::SerialDeviceCalibrator(rw::models::SerialDevice::Ptr device, const rw::kinematics::State& state,
		rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, DeviceJacobian::Ptr jacobian) :
		_device(device), _state(state), _referenceFrame(referenceFrame), _measurementFrame(measurementFrame), _jacobian(jacobian), _weight(true) {

}

SerialDeviceCalibrator::~SerialDeviceCalibrator() {

}

rw::kinematics::Frame::Ptr SerialDeviceCalibrator::getReferenceFrame() const {
	return _referenceFrame;
}

rw::kinematics::Frame::Ptr SerialDeviceCalibrator::getMeasurementFrame() const {
	return _measurementFrame;
}

DeviceJacobian::Ptr SerialDeviceCalibrator::getJacobian() const {
	return _jacobian;
}

void SerialDeviceCalibrator::setMeasurements(const std::vector<SerialDevicePoseMeasurement::Ptr>& measurements) {
	_measurements = measurements;
}

void SerialDeviceCalibrator::setWeight(bool weight) {
	_weight = weight;
}

int SerialDeviceCalibrator::getMinimumMeasurementCount() const {
	return ceil(_jacobian->getParameterCount() / 6);
}

void SerialDeviceCalibrator::calibrate() {
	Calibration::Ptr calibration = _jacobian->getCalibration();
	const bool wasApplied = calibration->isApplied();
	if (!wasApplied)
		calibration->apply();

	try {
		solve();
	} catch (rw::common::Exception& ex) {
		if (!wasApplied)
			calibration->revert();
		throw ex;
	}

	if (!wasApplied)
		calibration->revert();
}

void SerialDeviceCalibrator::computeJacobian(Eigen::MatrixXd& stackedJacobians) {
	computeJacobian(stackedJacobians, _measurements);
}

void SerialDeviceCalibrator::computeJacobian(Eigen::MatrixXd& stackedJacobians, const std::vector<SerialDevicePoseMeasurement::Ptr>& measurements) {
	const unsigned int measurementCount = measurements.size();
	const Calibration::Ptr calibration = _jacobian->getCalibration();
	const unsigned int parameterCount = _jacobian->getParameterCount();

	stackedJacobians.resize(6 * measurementCount, parameterCount);
	for (unsigned int measurementNo = 0; measurementNo < measurementCount; measurementNo++) {
		const rw::math::Q q = measurements[measurementNo]->getQ();
		_device->setQ(q, _state);

		// Update state to current
		calibration->correct(_state);

		// Setup Jacobian.
		stackedJacobians.block(6 * measurementNo, 0, 6, parameterCount) = _jacobian->compute(_referenceFrame, _measurementFrame, _state);

		// Weight system
		if (_weight) {
			const Eigen::Matrix<double, 6, 6> weightMatrix = Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> >(
					measurements[measurementNo]->getCovariance()).operatorInverseSqrt();
			stackedJacobians.block(6 * measurementNo, 0, 6, parameterCount) = weightMatrix * stackedJacobians.block(6 * measurementNo, 0, 6, parameterCount);
		}
	}
}

void SerialDeviceCalibrator::computeResiduals(Eigen::VectorXd& stackedResiduals) {
	computeResiduals(stackedResiduals, _measurements);
}

void SerialDeviceCalibrator::computeResiduals(Eigen::VectorXd& stackedResiduals, const std::vector<SerialDevicePoseMeasurement::Ptr>& measurements) {
	const unsigned int measurementCount = measurements.size();
	const Calibration::Ptr calibration = _jacobian->getCalibration();

	stackedResiduals.resize(6 * measurementCount);
	for (unsigned int measurementNo = 0; measurementNo < measurementCount; measurementNo++) {
		const rw::math::Q q = measurements[measurementNo]->getQ();
		_device->setQ(q, _state);

		// Update state to current
		calibration->correct(_state);

		// Prepare transformations.
		const Eigen::Affine3d tfmToMarker = Eigen::Affine3d(rw::kinematics::Kinematics::frameTframe(_referenceFrame.get(), _measurementFrame.get(), _state));

		// Setup residual vector.
		const Pose6D<double> pose = measurements[measurementNo]->getPose();
		stackedResiduals.segment<3>(6 * measurementNo) = -(pose.translation() - tfmToMarker.translation());
		const Eigen::Matrix3d dR = pose.rotation() * tfmToMarker.linear().transpose();
		stackedResiduals(6 * measurementNo + 3) = -(dR(2, 1) - dR(1, 2)) / 2;
		stackedResiduals(6 * measurementNo + 4) = -(dR(0, 2) - dR(2, 0)) / 2;
		stackedResiduals(6 * measurementNo + 5) = -(dR(1, 0) - dR(0, 1)) / 2;

		if (_weight) {
			const Eigen::Matrix<double, 6, 6> weightMatrix =
					Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> >(measurements[measurementNo]->getCovariance()).operatorInverseSqrt();
			stackedResiduals.segment<6>(6 * measurementNo) = weightMatrix * stackedResiduals.segment<6>(6 * measurementNo);
		}
	}
}

void SerialDeviceCalibrator::takeStep(const Eigen::VectorXd& step) {
	_jacobian->step(step);
}

}
}
