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
		_frame(frame), _isPreCorrection(isPreCorrection), _transform(transform), _enabledParameters(Eigen::Matrix<int, 6, 1>::Ones()) {

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

void FixedFrameCalibration::setEnabledParameters(bool x, bool y, bool z, bool roll, bool pitch, bool yaw) {
	_enabledParameters << x, y, z, roll, pitch, yaw;
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
	return _enabledParameters.sum();
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

	Eigen::MatrixXd jacobian(6, _enabledParameters.sum());
	int columnNo = 0;
	if (_enabledParameters(0)) {
		jacobian.block<3, 1>(0, columnNo) = rtmToPreCorrection.col(0);
		jacobian.block<3, 1>(3, columnNo) = Eigen::Vector3d::Zero();
		columnNo++;
	}
	if (_enabledParameters(1)) {
		jacobian.block<3, 1>(0, columnNo) = rtmToPreCorrection.col(1);
		jacobian.block<3, 1>(3, columnNo) = Eigen::Vector3d::Zero();
		columnNo++;
	}
	if (_enabledParameters(2)) {
		jacobian.block<3, 1>(0, columnNo) = rtmToPreCorrection.col(2);
		jacobian.block<3, 1>(3, columnNo) = Eigen::Vector3d::Zero();
		columnNo++;
	}
	if (_enabledParameters(3)) {
		jacobian.block<3, 1>(0, columnNo) = rtmToPreCorrection.col(0).cross(tlPreToEnd);
		jacobian.block<3, 1>(3, columnNo) = rtmToPreCorrection.col(0);
		columnNo++;
	}
	if (_enabledParameters(4)) {
		jacobian.block<3, 1>(0, columnNo) = rtmToPreCorrection.col(1).cross(tlPreToEnd);
		jacobian.block<3, 1>(3, columnNo) = rtmToPreCorrection.col(1);
		columnNo++;
	}
	if (_enabledParameters(5)) {
		jacobian.block<3, 1>(0, columnNo) = rtmToPreCorrection.col(2).cross(tlPreToEnd);
		jacobian.block<3, 1>(3, columnNo) = rtmToPreCorrection.col(2);
		columnNo++;
	}

	return jacobian;
}

void FixedFrameCalibration::doStep(const Eigen::VectorXd& step) {
	Pose6D<double> stepPose = Pose6D<double>::Zero();
	unsigned int enabledParameterNo = 0;
	for (int parameterNo = 0; parameterNo < _enabledParameters.rows(); parameterNo++)
		if (_enabledParameters(parameterNo)) {
			stepPose(parameterNo) = stepPose(parameterNo) + step(enabledParameterNo);
			enabledParameterNo++;
		}

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
