/*
 * FixedFrameJacobian.cpp
 *
 *  Created on: Nov 20, 2012
 *      Author: bing
 */

#include "FixedFrameJacobian.hpp"

#include <rw/kinematics.hpp>
#include <QtCore>

namespace rwlibs {
namespace calibration {

FixedFrameJacobian::FixedFrameJacobian(FixedFrameCalibration::Ptr calibration) :
	JacobianBase(6), _calibration(calibration) {
	RW_ASSERT(!calibration.isNull());
}

FixedFrameJacobian::~FixedFrameJacobian() {

}

Eigen::MatrixXd FixedFrameJacobian::doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame,
		const rw::kinematics::State& state) {
	RW_ASSERT(!referenceFrame.isNull());
	RW_ASSERT(!targetFrame.isNull());
	RW_ASSERT(_calibration->isApplied());

	const rw::kinematics::FixedFrame::Ptr fixedFrame = _calibration->getFrame();
	const bool isPostCorrection = _calibration->isPostCorrection();
	const Eigen::Affine3d toPreCorrectionTransform = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(),
			isPostCorrection ? fixedFrame->getParent(state) : fixedFrame.get(), state);
	const Eigen::Affine3d postCorrectionTransform = rw::kinematics::Kinematics::frameTframe(isPostCorrection ? fixedFrame->getParent(state) : fixedFrame.get(),
			targetFrame.get(), state);
	const Eigen::Matrix3d toPreCorrectionRotation = toPreCorrectionTransform.linear();
	const Eigen::Vector3d preToEndTranslation = (toPreCorrectionTransform * postCorrectionTransform).translation() - toPreCorrectionTransform.translation();

	const int columnCount = getParameterCount();
	Eigen::MatrixXd jacobian(6, columnCount);
	int columnIndex = 0;
	if (isParameterEnabled(PARAMETER_X)) {
		jacobian.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(0);
		jacobian.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
		columnIndex++;
	}
	if (isParameterEnabled(PARAMETER_Y)) {
		jacobian.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(1);
		jacobian.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
		columnIndex++;
	}
	if (isParameterEnabled(PARAMETER_Z)) {
		jacobian.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(2);
		jacobian.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
		columnIndex++;
	}
	if (isParameterEnabled(PARAMETER_ROLL)) {
		jacobian.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(2).cross(preToEndTranslation);
		jacobian.block<3, 1>(3, columnIndex) = toPreCorrectionRotation.col(2);
		columnIndex++;
	}
	if (isParameterEnabled(PARAMETER_PITCH)) {
		jacobian.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(1).cross(preToEndTranslation);
		jacobian.block<3, 1>(3, columnIndex) = toPreCorrectionRotation.col(1);
		columnIndex++;
	}
	if (isParameterEnabled(PARAMETER_YAW)) {
		jacobian.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(0).cross(preToEndTranslation);
		jacobian.block<3, 1>(3, columnIndex) = toPreCorrectionRotation.col(0);
	}

	return jacobian;
}

void FixedFrameJacobian::doTakeStep(const Eigen::VectorXd& step) {
	RW_ASSERT(step.rows() == 6);
	RW_ASSERT(_calibration->isApplied());

	_calibration->revert();

	const bool isPostCorrection = _calibration->isPostCorrection();
	const rw::math::Transform3D<> correction = _calibration->getCorrectionTransform();
	const rw::math::Transform3D<> stepTransform(rw::math::Vector3D<>(step(0), step(1), step(2)), rw::math::RPY<>(step(3), step(4), step(5)).toRotation3D());
	const rw::math::Transform3D<> correctedCorrection = isPostCorrection ? (stepTransform * correction) : (correction * stepTransform);
	_calibration->setCorrectionTransform(correctedCorrection);

	_calibration->apply();
}

}
}
