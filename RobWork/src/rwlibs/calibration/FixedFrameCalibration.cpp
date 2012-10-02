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

FixedFrameCalibration::FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPreCorrection, const Eigen::Affine3d& transform) :
		_frame(frame), _isPreCorrection(isPreCorrection), _transform(transform), _lockedParameters(Eigen::Matrix<int, 6, 1>::Zero()) {

}

FixedFrameCalibration::~FixedFrameCalibration() {

}

rw::kinematics::FixedFrame::Ptr FixedFrameCalibration::getFrame() const {
	return _frame;
}

Eigen::Affine3d FixedFrameCalibration::getTransform() const {
	return _transform;
}

bool FixedFrameCalibration::isPreCorrection() const {
	return _isPreCorrection;
}

void FixedFrameCalibration::setLockedParameters(bool x, bool y, bool z, bool roll, bool pitch, bool yaw) {
	_lockedParameters << x, y, z, roll, pitch, yaw;
}

void FixedFrameCalibration::doApply() {
	Eigen::Affine3d newTransform =
			_isPreCorrection ? Eigen::Affine3d(_frame->getFixedTransform()) * _transform : _transform * _frame->getFixedTransform();
	_frame->setTransform(newTransform);
}

void FixedFrameCalibration::doRevert() {
	Eigen::Affine3d newTransform =
			_isPreCorrection ?
					Eigen::Affine3d(_frame->getFixedTransform()) * _transform.inverse() : _transform.inverse() * _frame->getFixedTransform();
	_frame->setTransform(newTransform);
}

void FixedFrameCalibration::doCorrect(rw::kinematics::State& state) {

}

int FixedFrameCalibration::doGetParameterCount() const {
	return _lockedParameters.rows() - _lockedParameters.sum();
}

Eigen::MatrixXd FixedFrameCalibration::doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
		const rw::kinematics::State& state) {
	// Convert RobWork transformations.
	const Eigen::Affine3d tfmToPreCorrection = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(),
			_isPreCorrection ? _frame.get() : _frame->getParent(state), state);
	const Eigen::Affine3d tfmPostCorrection = rw::kinematics::Kinematics::frameTframe(_isPreCorrection ? _frame.get() : _frame->getParent(state),
			measurementFrame.get(), state);

	// Prepare transformations.
	const Eigen::Matrix3d rtmToPreCorrection = tfmToPreCorrection.linear();
	const Eigen::Vector3d tlPreToEnd = (tfmToPreCorrection * tfmPostCorrection).translation() - tfmToPreCorrection.translation();

	const int columnCount = getParameterCount();
	Eigen::MatrixXd jacobian(6, columnCount);
	int columnIndex = 0;
	if (!_lockedParameters(0)) {
		jacobian.block<3, 1>(0, columnIndex) = rtmToPreCorrection.col(0);
		jacobian.block<3, 1>(3, columnIndex++) = Eigen::Vector3d::Zero();
	}
	if (!_lockedParameters(1)) {
		jacobian.block<3, 1>(0, columnIndex) = rtmToPreCorrection.col(1);
		jacobian.block<3, 1>(3, columnIndex++) = Eigen::Vector3d::Zero();
	}
	if (!_lockedParameters(2)) {
		jacobian.block<3, 1>(0, columnIndex) = rtmToPreCorrection.col(2);
		jacobian.block<3, 1>(3, columnIndex++) = Eigen::Vector3d::Zero();
	}
	if (!_lockedParameters(3)) {
		jacobian.block<3, 1>(0, columnIndex) = rtmToPreCorrection.col(0).cross(tlPreToEnd);
		jacobian.block<3, 1>(3, columnIndex++) = rtmToPreCorrection.col(0);
	}
	if (!_lockedParameters(4)) {
		jacobian.block<3, 1>(0, columnIndex) = rtmToPreCorrection.col(1).cross(tlPreToEnd);
		jacobian.block<3, 1>(3, columnIndex++) = rtmToPreCorrection.col(1);
	}
	if (!_lockedParameters(5)) {
		jacobian.block<3, 1>(0, columnIndex) = rtmToPreCorrection.col(2).cross(tlPreToEnd);
		jacobian.block<3, 1>(3, columnIndex) = rtmToPreCorrection.col(2);
	}

	return jacobian;
}

void FixedFrameCalibration::doTakeStep(const Eigen::VectorXd& step) {
	const int parameterCount = _lockedParameters.rows();
	Pose6D<double> stepPose = Pose6D<double>::Zero();
	unsigned int unlockedParameterIndex = 0;
	for (int parameterIndex = 0; parameterIndex < parameterCount; parameterIndex++)
		if (!_lockedParameters(parameterIndex))
			stepPose(parameterIndex) = stepPose(parameterIndex) + step(unlockedParameterIndex++);

	Eigen::Affine3d stepTransform = stepPose.toTransform();
	_transform = _isPreCorrection ? _transform * stepTransform : stepTransform * _transform;
	if (isApplied()) {
		Eigen::Affine3d correctedBaseTransform =
				_isPreCorrection ? Eigen::Affine3d(_frame->getFixedTransform()) * stepTransform : stepTransform * _frame->getFixedTransform();
		_frame->setTransform(correctedBaseTransform);
	}
}

}
}
