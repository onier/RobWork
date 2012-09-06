/*
 * FixedFrameJacobian.cpp
 *
 *  Created on: Aug 30, 2012
 *      Author: bing
 */

#include "FixedFrameJacobian.hpp"

#include "Pose6D.hpp"

namespace rwlibs {
namespace calibration {

FixedFrameJacobian::FixedFrameJacobian(FixedFrameCalibration::Ptr calibration) :
		_calibration(calibration), _enabledParameters(Eigen::Matrix<int, 6, 1>::Ones()) {

}

DeviceCalibration::Ptr FixedFrameJacobian::getCalibration() const {
	return _calibration;
}

int FixedFrameJacobian::getParameterCount() const {
	return _isEnabled ? _enabledParameters.sum() : 0;
}

Eigen::MatrixXd FixedFrameJacobian::compute(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
		const rw::kinematics::State& state) {
	if (!_isEnabled)
		RW_THROW("Not enabled.");
	if (!_enabledParameters.sum())
		RW_THROW("No parameters enabled.");

	const rw::kinematics::Frame::Ptr frame = _calibration->getFrame();
	const bool isPreCorrection = _calibration->isPreCorrection();

	// Convert RobWork transformations.
	const Eigen::Affine3d tfmToPreCorrection = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(),
			isPreCorrection ? frame.get() : frame->getParent(state), state);
	const Eigen::Affine3d tfmPostCorrection = rw::kinematics::Kinematics::frameTframe(isPreCorrection ? frame.get() : frame->getParent(state),
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

void FixedFrameJacobian::step(const Eigen::VectorXd& step) {
	if (!_isEnabled)
		RW_THROW("Not enabled.");
	if (!_enabledParameters.sum())
		RW_THROW("No parameters enabled.");

	Pose6D<double> pose = Pose6D<double>::Zero();
	unsigned int enabledParameterNo = 0;
	for (int parameterNo = 0; parameterNo < _enabledParameters.rows(); parameterNo++)
		if (_enabledParameters(parameterNo)) {
			pose(parameterNo) = pose(parameterNo) + step(enabledParameterNo);
			enabledParameterNo++;
		}

	_calibration->correct(pose.toTransform());
}

void FixedFrameJacobian::setEnabledParameters(bool x, bool y, bool z, bool roll, bool pitch, bool yaw) {
	_enabledParameters << x, y, z, roll, pitch, yaw;
}

}
}
