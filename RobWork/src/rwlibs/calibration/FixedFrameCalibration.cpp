/*
 * FixedFrameCalibration.cpp
 *
 *  Created on: Aug 28, 2012
 *      Author: bing
 */

#include "FixedFrameCalibration.hpp"

#include "Pose6D.hpp"
#include <rw/kinematics.hpp>
#include <QtCore>

namespace rwlibs {
namespace calibration {

FixedFrameCalibration::FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPostCorrection, const Eigen::Affine3d& transform) :
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

Eigen::Affine3d FixedFrameCalibration::getCorrection() const {
	return _correction;
}

void FixedFrameCalibration::setCorrection(const Eigen::Affine3d& correction) {
	_correction = correction;
}

void FixedFrameCalibration::setLockedParameters(bool x, bool y, bool z, bool roll, bool pitch, bool yaw) {
	_lockedParameters << x, y, z, roll, pitch, yaw;
}

void FixedFrameCalibration::doApply() {
	Eigen::Affine3d correctedTransform =
			_isPostCorrection ? _correction * _frame->getFixedTransform() : Eigen::Affine3d(_frame->getFixedTransform()) * _correction;
	_frame->setTransform(correctedTransform);
}

void FixedFrameCalibration::doRevert() {
	Eigen::Affine3d correctedTransform =
			_isPostCorrection ? _correction.inverse() * _frame->getFixedTransform() : Eigen::Affine3d(_frame->getFixedTransform()) * _correction.inverse();
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
		jacobian.block<3, 1>(3, columnIndex++) = Eigen::Vector3d::Zero();
	}
	if (!_lockedParameters(PARAMETER_Y)) {
		jacobian.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(1);
		jacobian.block<3, 1>(3, columnIndex++) = Eigen::Vector3d::Zero();
	}
	if (!_lockedParameters(PARAMETER_Z)) {
		jacobian.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(2);
		jacobian.block<3, 1>(3, columnIndex++) = Eigen::Vector3d::Zero();
	}
	if (!_lockedParameters(PARAMETER_ROLL)) {
		jacobian.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(0).cross(preToEndTranslation);
		jacobian.block<3, 1>(3, columnIndex++) = toPreCorrectionRotation.col(0);
	}
	if (!_lockedParameters(PARAMETER_PITCH)) {
		jacobian.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(1).cross(preToEndTranslation);
		jacobian.block<3, 1>(3, columnIndex++) = toPreCorrectionRotation.col(1);
	}
	if (!_lockedParameters(PARAMETER_YAW)) {
		jacobian.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(2).cross(preToEndTranslation);
		jacobian.block<3, 1>(3, columnIndex) = toPreCorrectionRotation.col(2);
	}

	return jacobian;
}

void FixedFrameCalibration::doTakeStep(const Eigen::VectorXd& step) {
	const int parameterCount = _lockedParameters.rows();
	Pose6Dd stepPose = Pose6Dd::Zero();
	unsigned int unlockedParameterIndex = 0;
	for (int parameterIndex = 0; parameterIndex < parameterCount; parameterIndex++)
		if (!_lockedParameters(parameterIndex))
			stepPose(parameterIndex) = stepPose(parameterIndex) + step(unlockedParameterIndex++);

	Eigen::Affine3d stepTransform = stepPose.toTransform();
	_correction = _isPostCorrection ? stepTransform * _correction : _correction * stepTransform;
	if (isApplied()) {
		Eigen::Affine3d correctedTransform =
				_isPostCorrection ? stepTransform * _frame->getFixedTransform() : Eigen::Affine3d(_frame->getFixedTransform()) * stepTransform;
		_frame->setTransform(correctedTransform);
	}
}

}
}
