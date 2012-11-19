/*
 * SerialDeviceCalibrator.cpp
 *
 *  Created on: Feb 27, 2012
 *      Author: bing
 */

#include "SerialDeviceCalibrator.hpp"

#include "nlls/NLLSSolver.hpp"
#include <Eigen/Eigenvalues>

namespace rwlibs {
namespace calibration {

SerialDeviceCalibrator::SerialDeviceCalibrator(rw::models::SerialDevice::Ptr device,
		rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, Calibration::Ptr calibration) :
		_device(device), _referenceFrame(referenceFrame), _measurementFrame(measurementFrame), _calibration(calibration), _isWeightingEnabled(true) {

}

SerialDeviceCalibrator::~SerialDeviceCalibrator() {

}

rw::kinematics::Frame::Ptr SerialDeviceCalibrator::getReferenceFrame() const {
	return _referenceFrame;
}

void SerialDeviceCalibrator::setReferenceFrame(rw::kinematics::Frame::Ptr referenceFrame) {
	_referenceFrame = referenceFrame;
}

rw::kinematics::Frame::Ptr SerialDeviceCalibrator::getMeasurementFrame() const {
	return _measurementFrame;
}

void SerialDeviceCalibrator::setMeasurementFrame(rw::kinematics::Frame::Ptr measurementFrame) {
	_measurementFrame = measurementFrame;
}

Calibration::Ptr SerialDeviceCalibrator::getCalibration() const {
	return _calibration;
}

unsigned int SerialDeviceCalibrator::getMinimumMeasurementCount() const {
	return ceil(float(_calibration->getParameterCount()) / 6);
}

const std::vector<SerialDevicePoseMeasurement::Ptr>& SerialDeviceCalibrator::getMeasurements() const {
	return _measurements;
}

void SerialDeviceCalibrator::addMeasurement(SerialDevicePoseMeasurement::Ptr measurement) {
	_measurements.push_back(measurement);
}

void SerialDeviceCalibrator::addMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform, const Eigen::Matrix<double, 6, 6>& covarianceMatrix) {
	_measurements.push_back(rw::common::ownedPtr(new SerialDevicePoseMeasurement(q, transform, covarianceMatrix)));
}

void SerialDeviceCalibrator::setMeasurements(const std::vector<SerialDevicePoseMeasurement::Ptr>& measurements) {
	_measurements = measurements;
}

bool SerialDeviceCalibrator::isWeightingEnabled() const {
	return _isWeightingEnabled;
}

void SerialDeviceCalibrator::setWeightingEnabled(bool isWeightingEnabled) {
	_isWeightingEnabled = isWeightingEnabled;
}

void SerialDeviceCalibrator::calibrate(const rw::kinematics::State& state) {
	RW_ASSERT(!_calibration->isLocked());

	_state = state;

	// Apply calibration if not applied.
	const bool wasApplied = _calibration->isApplied();
	if (!wasApplied)
		_calibration->apply();

	// Solve non-linear least square system.
	_solver = rw::common::ownedPtr(new NLLSSolver(this));
	try {
		_solver->solve();
	} catch (rw::common::Exception& ex) {
		// Revert calibration if it was not applied.
		if (!wasApplied)
			_calibration->revert();

		// Re-trow exception.
		throw ex;
	}

	// Revert calibration if it was not applied.
	if (!wasApplied)
		_calibration->revert();
}

NLLSSolverLog::Ptr SerialDeviceCalibrator::getSolverLog() const {
	RW_ASSERT(!_solver.isNull());
	return _solver->getLog();
}

Eigen::MatrixXd SerialDeviceCalibrator::estimateCovariance() const {
	RW_ASSERT(!_solver.isNull());
	return _solver->estimateCovariance();
}

void SerialDeviceCalibrator::computeJacobian(Eigen::MatrixXd& stackedJacobians) {
	const unsigned int measurementCount = _measurements.size();
	const unsigned int parameterCount = _calibration->getParameterCount();

	stackedJacobians.resize(6 * measurementCount, parameterCount);
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		const SerialDevicePoseMeasurement::Ptr measurement = _measurements[measurementIndex];
		
		// Update state according to current measurement.
		const rw::math::Q q = measurement->getQ();
		_device->setQ(q, _state);
		_calibration->correct(_state);

		// Compute Jacobian.
		stackedJacobians.block(6 * measurementIndex, 0, 6, parameterCount) = _calibration->computeJacobian(_referenceFrame, _measurementFrame, _state);
		
		// Weight Jacobian according to covariances.
		if (_isWeightingEnabled && measurement->hasCovariance()) {
			const Eigen::Matrix<double, 6, 6> weightMatrix = Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> >(
					measurement->getCovariance()).operatorInverseSqrt();
			stackedJacobians.block(6 * measurementIndex, 0, 6, parameterCount) = weightMatrix
					* stackedJacobians.block(6 * measurementIndex, 0, 6, parameterCount);
		}
	}
}

void SerialDeviceCalibrator::computeResiduals(Eigen::VectorXd& stackedResiduals) {
	const unsigned int measurementCount = _measurements.size();

	stackedResiduals.resize(6 * measurementCount);
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		const SerialDevicePoseMeasurement::Ptr measurement = _measurements[measurementIndex];
		
		// Update state according to current measurement.
		const rw::math::Q q = measurement->getQ();
		_device->setQ(q, _state);
		_calibration->correct(_state);
		
		// Compute residuals.
		const rw::math::Transform3D<> tfmMeasurement = measurement->getTransform();
		const rw::math::Transform3D<> tfmModel = rw::kinematics::Kinematics::frameTframe(_referenceFrame.get(), _measurementFrame.get(), _state);
		const rw::math::Vector3D<> dP = tfmModel.P() - tfmMeasurement.P();
		stackedResiduals(6 * measurementIndex + 0) = dP(0);
		stackedResiduals(6 * measurementIndex + 1) = dP(1);
		stackedResiduals(6 * measurementIndex + 2) = dP(2);
		const rw::math::Rotation3D<> dR = tfmModel.R() * rw::math::inverse(tfmMeasurement.R());
		stackedResiduals(6 * measurementIndex + 3) = (dR(2, 1) - dR(1, 2)) / 2;
		stackedResiduals(6 * measurementIndex + 4) = (dR(0, 2) - dR(2, 0)) / 2;
		stackedResiduals(6 * measurementIndex + 5) = (dR(1, 0) - dR(0, 1)) / 2;

		// Weight residuals according to covariances.
		if (_isWeightingEnabled && measurement->hasCovariance()) {
			const Eigen::Matrix<double, 6, 6> weightMatrix = Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> >(
					measurement->getCovariance()).operatorInverseSqrt();
			stackedResiduals.segment<6>(6 * measurementIndex) = weightMatrix * stackedResiduals.segment<6>(6 * measurementIndex);
		}
	}
}

void SerialDeviceCalibrator::takeStep(const Eigen::VectorXd& step) {
	_calibration->takeStep(step);
}

}
}
