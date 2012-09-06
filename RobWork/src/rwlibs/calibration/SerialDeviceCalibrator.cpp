/*
 * SerialDeviceCalibrator.cpp
 *
 *  Created on: Feb 27, 2012
 *      Author: bing
 */

#include "SerialDeviceCalibrator.hpp"

#include <Eigen/Eigenvalues>
#include <rw/kinematics.hpp>
#include <rw/models/DHParameterSet.hpp>

namespace rwlibs {
namespace calibration {

SerialDeviceCalibrator::SerialDeviceCalibrator(rw::models::SerialDevice::Ptr device,
		rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, DeviceJacobian::Ptr jacobian) :
		_device(device), _referenceFrame(referenceFrame), _measurementFrame(measurementFrame), _jacobian(jacobian), _weight(true), _maxIterations(100), _precision(
				1e-14) {

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

void SerialDeviceCalibrator::setMeasurementList(const SerialDevicePoseMeasurementList& measurements) {
	_measurements = measurements;
}

void SerialDeviceCalibrator::setWeight(bool weight) {
	_weight = weight;
}

int SerialDeviceCalibrator::getMinimumMeasurementCount() const {
	return ceil(_jacobian->getParameterCount() / 6);
}

void SerialDeviceCalibrator::calibrate(const rw::kinematics::State& state) {
	DeviceCalibration::Ptr calibration = _jacobian->getCalibration();
	const bool wasApplied = calibration->isApplied();
	if (!wasApplied)
		calibration->apply();

	try {
		int iterationNo = 0;
		std::vector<Eigen::VectorXd> steps;
		Eigen::MatrixXd jacobian;
		Eigen::VectorXd residuals;
		while (true) {
			iterationNo++;

			computeJacobian(jacobian, state, _measurements);
			computeResiduals(residuals, state, _measurements);

			Eigen::JacobiSVD<Eigen::MatrixXd> jacobianSvd = jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

			Eigen::VectorXd step = jacobianSvd.solve(-residuals);
			steps.push_back(step);

			// Compute singularity-test, norm of step and residuals, rate of convergence and condition number.
			bool isSingular = (jacobianSvd.singularValues().rows() != jacobianSvd.nonzeroSingularValues());
			double residualsNorm = residuals.norm();
			double stepNorm = step.norm();
			double stepErrorNorm = (jacobian * step - residuals).norm();
			double roc = std::numeric_limits<double>::signaling_NaN();
			if (iterationNo > 2) {
				roc = log(steps[iterationNo - 1].norm() / steps[iterationNo - 2].norm()) / log(steps[iterationNo - 2].norm() / steps[iterationNo - 3].norm());
			}
			Eigen::VectorXd singularValues = jacobianSvd.singularValues();
			double conditionNumber = singularValues(0) / singularValues(singularValues.rows() - 1);

//			std::cout << "---" << std::endl;
//			std::cout << "Jacobian (" << jacobian.rows() << "x" << jacobian.cols() << "):" << std::endl;
//			std::cout << jacobian.block(0, 0, jacobian.rows() > 5 ? 6 : jacobian.rows(), jacobian.cols()) << std::endl;
//			std::cout << "Residuals:\t" << residuals.segment(0, residuals.rows() > 6 ? 12 : 6).transpose() << std::endl;
//			std::cout << "Step: \t" << step.transpose() << std::endl;
			std::cout << "Iteration " << iterationNo << " completed. Singular: " << (isSingular ? "Yes" : "No") << ". Condition: " << conditionNumber
					<< ". ||Residuals||: " << residualsNorm << ". ||Step||: " << stepNorm << ". ||StepError||: " << stepErrorNorm << ". RoC: " << roc << "."
					<< std::endl;
//			std::cin.ignore();

			if (isSingular)
				RW_THROW("Singular Jacobian.");
			if (isnan(stepNorm))
				RW_THROW("NaN step.");
			if (isinf(stepNorm))
				RW_THROW("Infinite step.");
			if (_maxIterations > 0 && iterationNo >= _maxIterations)
				RW_THROW("Max iterations reached.");

			_jacobian->step(step);

			if (stepNorm <= _precision) {
				computeJacobian(jacobian, state, _measurements);
				computeResiduals(residuals, state, _measurements);
				break;
			}
		}
	} catch (rw::common::Exception& ex) {
		if (!wasApplied)
			calibration->revert();
		throw ex;
	}

	if (!wasApplied)
		calibration->revert();
}

void SerialDeviceCalibrator::computeJacobian(Eigen::MatrixXd& stackedJacobians, rw::kinematics::State state) {
	computeJacobian(stackedJacobians, state, _measurements);
}

void SerialDeviceCalibrator::computeJacobian(Eigen::MatrixXd& stackedJacobians, rw::kinematics::State state,
		const SerialDevicePoseMeasurementList& measurements) {
	const unsigned int measurementCount = measurements.size();
	const unsigned int parameterCount = _jacobian->getParameterCount();

	stackedJacobians.resize(6 * measurementCount, parameterCount);
	for (unsigned int measurementNo = 0; measurementNo < measurementCount; measurementNo++) {
		rw::math::Q q = measurements[measurementNo].getQ();
		_device->setQ(q, state);

		// Update state to current
		_jacobian->getCalibration()->correct(state);

		// Setup Jacobian.
		stackedJacobians.block(6 * measurementNo, 0, 6, parameterCount) = _jacobian->compute(_referenceFrame, _measurementFrame, state);

		// Weight system
		if (_weight) {
			Eigen::Matrix<double, 6, 6> weightMatrix =
					Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> >(measurements[measurementNo].getCovariance()).operatorInverseSqrt();
			stackedJacobians.block(6 * measurementNo, 0, 6, parameterCount) = weightMatrix * stackedJacobians.block(6 * measurementNo, 0, 6, parameterCount);
		}
	}
}

void SerialDeviceCalibrator::computeResiduals(Eigen::VectorXd& stackedResiduals, rw::kinematics::State state) {
	computeResiduals(stackedResiduals, state, _measurements);
}

void SerialDeviceCalibrator::computeResiduals(Eigen::VectorXd& stackedResiduals, rw::kinematics::State state,
		const SerialDevicePoseMeasurementList& measurements) {
	const unsigned int measurementCount = measurements.size();

	stackedResiduals.resize(6 * measurementCount);
	for (unsigned int measurementNo = 0; measurementNo < measurementCount; measurementNo++) {
		rw::math::Q q = measurements[measurementNo].getQ();
		_device->setQ(q, state);

		// Update state to current
		_jacobian->getCalibration()->correct(state);

		// Prepare transformations.
		const Eigen::Affine3d tfmToMarker = Eigen::Affine3d(rw::kinematics::Kinematics::frameTframe(_referenceFrame.get(), _measurementFrame.get(), state));

		// Setup residual vector.
		Pose6D<double> pose = measurements[measurementNo].getPose();
		stackedResiduals.segment<3>(6 * measurementNo) = -(pose.translation() - tfmToMarker.translation());
		Eigen::Matrix3d dR = pose.rotation() * tfmToMarker.linear().transpose();
		stackedResiduals(6 * measurementNo + 3) = -(dR(2, 1) - dR(1, 2)) / 2;
		stackedResiduals(6 * measurementNo + 4) = -(dR(0, 2) - dR(2, 0)) / 2;
		stackedResiduals(6 * measurementNo + 5) = -(dR(1, 0) - dR(0, 1)) / 2;

		if (_weight) {
			Eigen::Matrix<double, 6, 6> weightMatrix =
					Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> >(measurements[measurementNo].getCovariance()).operatorInverseSqrt();
			stackedResiduals.segment<6>(6 * measurementNo) = weightMatrix * stackedResiduals.segment<6>(6 * measurementNo);
		}
	}
}

}
}
