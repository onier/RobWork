/*
 * FixedFrameCalibration.cpp
 *
 *  Created on: Aug 28, 2012
 *      Author: bing
 */

#include "FixedFrameCalibration.hpp"

#include <rw/kinematics.hpp>
#include <QtCore>

namespace rwlibs {
namespace calibration {

FixedFrameCalibration::FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame) :
		_frame(frame), _isPostCorrection(false), _correction(Eigen::Affine3d::Identity()), _lockedParameters(Eigen::Matrix<int, 6, 1>::Zero()) {

}

FixedFrameCalibration::FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPostCorrection) :
		_frame(frame), _isPostCorrection(isPostCorrection), _correction(Eigen::Affine3d::Identity()), _lockedParameters(Eigen::Matrix<int, 6, 1>::Zero()) {

}

FixedFrameCalibration::FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPostCorrection, const rw::math::Transform3D<>& transform) :
		_frame(frame), _isPostCorrection(isPostCorrection), _correction(transform), _lockedParameters(Eigen::Matrix<int, 6, 1>::Zero()) {

}

FixedFrameCalibration::~FixedFrameCalibration() {

}

rw::kinematics::FixedFrame::Ptr FixedFrameCalibration::getFrame() const {
	return _frame;
}

bool FixedFrameCalibration::isPostCorrection() const {
	return _isPostCorrection;
}

rw::math::Transform3D<> FixedFrameCalibration::getCorrection() const {
	return _correction;
}

void FixedFrameCalibration::setCorrection(const rw::math::Transform3D<>& correction) {
	_correction = correction;
}

bool FixedFrameCalibration::isParameterLocked(int parameterIndex) {
	RW_ASSERT(parameterIndex < _lockedParameters.size());
	return _lockedParameters(parameterIndex);
}

void FixedFrameCalibration::setParameterLocked(int parameterIndex, bool isLocked) {
	RW_ASSERT(parameterIndex < _lockedParameters.size());
	_lockedParameters(parameterIndex) = isLocked;
}

void FixedFrameCalibration::doApply() {
	rw::math::Transform3D<> correctedTransform =
			_isPostCorrection ? _correction * _frame->getFixedTransform() : _frame->getFixedTransform() * _correction;
	_frame->setTransform(correctedTransform);
}

void FixedFrameCalibration::doRevert() {
	rw::math::Transform3D<> correctedTransform =
			_isPostCorrection ? rw::math::inverse(_correction) * _frame->getFixedTransform() : _frame->getFixedTransform() * rw::math::inverse(_correction);
	_frame->setTransform(correctedTransform);
}

void FixedFrameCalibration::doCorrect(rw::kinematics::State& state) {

}

int FixedFrameCalibration::doGetParameterCount() const {
	return _lockedParameters.rows() - _lockedParameters.sum();
}

Eigen::MatrixXd FixedFrameCalibration::doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame,
		const rw::kinematics::State& state) {
	// Convert RobWork transformations.
	const Eigen::Affine3d toPreCorrectionTransform = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(),
			_isPostCorrection ? _frame->getParent(state) : _frame.get(), state);
	const Eigen::Affine3d postCorrectionTransform = rw::kinematics::Kinematics::frameTframe(_isPostCorrection ? _frame->getParent(state) : _frame.get(),
			targetFrame.get(), state);

	// Prepare transformations.
	const Eigen::Matrix3d toPreCorrectionRotation = toPreCorrectionTransform.linear();
	const Eigen::Vector3d preToEndTranslation = (toPreCorrectionTransform * postCorrectionTransform).translation() - toPreCorrectionTransform.translation();

	const int columnCount = getParameterCount();
	Eigen::MatrixXd jacobian(6, columnCount);
	int columnIndex = 0;
	if (!_lockedParameters(PARAMETER_X)) {
		jacobian.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(0);
		jacobian.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
		columnIndex++;
	}
	if (!_lockedParameters(PARAMETER_Y)) {
		jacobian.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(1);
		jacobian.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
		columnIndex++;
	}
	if (!_lockedParameters(PARAMETER_Z)) {
		jacobian.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(2);
		jacobian.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
		columnIndex++;
	}
	if (!_lockedParameters(PARAMETER_ROLL)) {
		jacobian.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(2).cross(preToEndTranslation);
		jacobian.block<3, 1>(3, columnIndex) = toPreCorrectionRotation.col(2);
		columnIndex++;
	}
	if (!_lockedParameters(PARAMETER_PITCH)) {
		jacobian.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(1).cross(preToEndTranslation);
		jacobian.block<3, 1>(3, columnIndex) = toPreCorrectionRotation.col(1);
		columnIndex++;
	}
	if (!_lockedParameters(PARAMETER_YAW)) {
		jacobian.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(0).cross(preToEndTranslation);
		jacobian.block<3, 1>(3, columnIndex) = toPreCorrectionRotation.col(0);
	}

	return jacobian;
}

void FixedFrameCalibration::doTakeStep(const Eigen::VectorXd& step) {
	const int parameterCount = _lockedParameters.rows();
	Eigen::Matrix<double, 6, 1> mappedStep = Eigen::Matrix<double, 6, 1>::Zero();
	unsigned int unlockedParameterIndex = 0;
	for (int parameterIndex = 0; parameterIndex < parameterCount; parameterIndex++) {
		if (!isParameterLocked(parameterIndex)) {
			mappedStep(parameterIndex) = step(unlockedParameterIndex);
			unlockedParameterIndex++;
		}
	}

	rw::math::Transform3D<> stepTransform(rw::math::Vector3D<>(mappedStep(0), mappedStep(1), mappedStep(2)), rw::math::RPY<>(mappedStep(3), mappedStep(4), mappedStep(5)).toRotation3D());

	_correction = _isPostCorrection ? stepTransform * _correction : _correction * stepTransform;
	if (isApplied()) {
		rw::math::Transform3D<> correctedTransform =
			_isPostCorrection ? stepTransform * _frame->getFixedTransform() : _frame->getFixedTransform() * stepTransform;
		_frame->setTransform(correctedTransform);
	}
}

}
}
