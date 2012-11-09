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

void SerialDeviceCalibrator::addMeasurement(const rw::math::Q& q, const Pose6D<double>& pose, const Eigen::Matrix<double, 6, 6>& covarianceMatrix) {
	_measurements.push_back(rw::common::ownedPtr(new SerialDevicePoseMeasurement(q, pose, covarianceMatrix)));
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

NLLSSolverLog::Ptr SerialDeviceCalibrator::getSolverLog() const {
	return _solverLog;
}

void SerialDeviceCalibrator::calibrate(const rw::kinematics::State& state) {
	_state = state;

	// Throw exception if calibration is locked.
	if (_calibration->isLocked())
		RW_THROW("Calibration is locked.");

	// Apply calibration if not applied.
	const bool wasApplied = _calibration->isApplied();
	if (!wasApplied)
		_calibration->apply();

	// Solve non-linear least square system.
	NLLSSolver::Ptr solver(rw::common::ownedPtr(new NLLSSolver(this)));
	try {
		solver->solve();
		_covarianceMatrix = solver->estimateCovarianceMatrix();

		// Get solver log.
		_solverLog = solver->getLog();
	} catch (rw::common::Exception& ex) {
		// Get solver log.
		_solverLog = solver->getLog();

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

Eigen::MatrixXd SerialDeviceCalibrator::getCovarianceMatrix() const {
	return _covarianceMatrix;
}

void SerialDeviceCalibrator::computeJacobian(Eigen::MatrixXd& stackedJacobians) {
	computeJacobian(stackedJacobians, _measurements);
}

void SerialDeviceCalibrator::computeJacobian(Eigen::MatrixXd& stackedJacobians, const std::vector<SerialDevicePoseMeasurement::Ptr>& measurements) {
	const unsigned int measurementCount = measurements.size();
	const unsigned int parameterCount = _calibration->getParameterCount();

	stackedJacobians.resize(6 * measurementCount, parameterCount);
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		const SerialDevicePoseMeasurement::Ptr measurement = measurements[measurementIndex];

		const rw::math::Q q = measurement->getQ();
		_device->setQ(q, _state);

		// Update state to current
		_calibration->correct(_state);

		// Setup Jacobian.
		stackedJacobians.block(6 * measurementIndex, 0, 6, parameterCount) = _calibration->computeJacobian(_referenceFrame, _measurementFrame, _state);

		// Weight system
		if (_isWeightingEnabled && measurement->hasCovariance()) {
			const Eigen::Matrix<double, 6, 6> weightMatrix = Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> >(
					measurement->getCovariance()).operatorInverseSqrt();
			stackedJacobians.block(6 * measurementIndex, 0, 6, parameterCount) = weightMatrix
					* stackedJacobians.block(6 * measurementIndex, 0, 6, parameterCount);
		}
	}
}

void SerialDeviceCalibrator::computeResiduals(Eigen::VectorXd& stackedResiduals) {
	computeResiduals(stackedResiduals, _measurements);
}

void SerialDeviceCalibrator::computeResiduals(Eigen::VectorXd& stackedResiduals, const std::vector<SerialDevicePoseMeasurement::Ptr>& measurements) {
	const unsigned int measurementCount = measurements.size();

	stackedResiduals.resize(6 * measurementCount);
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		const SerialDevicePoseMeasurement::Ptr measurement = measurements[measurementIndex];

		const rw::math::Q q = measurement->getQ();
		// Update state to current
		_device->setQ(q, _state);
		if (_calibration->isApplied())
			_calibration->correct(_state);

		const Eigen::Affine3d tfmMeasurement = measurement->getPose().toTransform();
		const Eigen::Affine3d tfmModel = Eigen::Affine3d(rw::kinematics::Kinematics::frameTframe(_referenceFrame.get(), _measurementFrame.get(), _state));
		const Eigen::Affine3d dT = tfmModel.difference(tfmMeasurement);

		// Setup residual vector.
		stackedResiduals.segment<3>(6 * measurementIndex) = dT.translation();
		const Eigen::Matrix3d dR = dT.linear();
		stackedResiduals(6 * measurementIndex + 3) = (dR(2, 1) - dR(1, 2)) / 2;
		stackedResiduals(6 * measurementIndex + 4) = (dR(0, 2) - dR(2, 0)) / 2;
		stackedResiduals(6 * measurementIndex + 5) = (dR(1, 0) - dR(0, 1)) / 2;

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