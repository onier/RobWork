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
			JacobianBase(calibration), _calibration(calibration) {

		}

		FixedFrameJacobian::~FixedFrameJacobian() {

		}

		Eigen::MatrixXd FixedFrameJacobian::doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame,
			const rw::kinematics::State& state) {
				const rw::kinematics::FixedFrame::Ptr fixedFrame = _calibration->getFrame();
				const bool isPostCorrection = _calibration->isPostCorrection();
				const Eigen::Affine3d correctionTransform = _calibration->getCorrectionTransform();

				const rw::kinematics::Frame* preCorrectionFrame = isPostCorrection ? fixedFrame.get() : fixedFrame->getParent(state);
				Eigen::Affine3d toPreCorrectionTransform = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), preCorrectionFrame, state);
				if (isPostCorrection)
					toPreCorrectionTransform = toPreCorrectionTransform * correctionTransform.inverse();
				Eigen::Affine3d postCorrectionTransform = rw::kinematics::Kinematics::frameTframe(preCorrectionFrame, targetFrame.get(), state);
				if (!isPostCorrection)
					postCorrectionTransform = correctionTransform.inverse() * postCorrectionTransform;
				const Eigen::Matrix3d toPreCorrectionRotation = toPreCorrectionTransform.linear();
				const Eigen::Vector3d preToEndTranslation = (toPreCorrectionTransform * correctionTransform * postCorrectionTransform).translation() - toPreCorrectionTransform.translation();

				const int columnCount = getColumnCount();
				Eigen::MatrixXd jacobianMatrix(6, columnCount);
				int columnIndex = 0;
				if (_calibration->isParameterEnabled(FixedFrameCalibration::PARAMETER_X)) {
					jacobianMatrix.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(0);
					jacobianMatrix.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
					columnIndex++;
				}
				if (_calibration->isParameterEnabled(FixedFrameCalibration::PARAMETER_Y)) {
					jacobianMatrix.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(1);
					jacobianMatrix.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
					columnIndex++;
				}
				if (_calibration->isParameterEnabled(FixedFrameCalibration::PARAMETER_Z)) {
					jacobianMatrix.block<3, 1>(0, columnIndex) = toPreCorrectionRotation.col(2);
					jacobianMatrix.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
					columnIndex++;
				}
				double rollAngle = _calibration->getParameterValue(FixedFrameCalibration::PARAMETER_ROLL);
				Eigen::Matrix3d localRotation = toPreCorrectionRotation * Eigen::AngleAxisd(rollAngle, Eigen::Vector3d::UnitZ());
				if (_calibration->isParameterEnabled(FixedFrameCalibration::PARAMETER_ROLL)) {
					jacobianMatrix.block<3, 1>(0, columnIndex) = localRotation.col(2).cross(preToEndTranslation);
					jacobianMatrix.block<3, 1>(3, columnIndex) = localRotation.col(2);
					columnIndex++;
				}
				double pitchAngle = _calibration->getParameterValue(FixedFrameCalibration::PARAMETER_PITCH);
				localRotation = localRotation * Eigen::AngleAxisd(pitchAngle, Eigen::Vector3d::UnitY());
				if (_calibration->isParameterEnabled(FixedFrameCalibration::PARAMETER_PITCH)) {
					jacobianMatrix.block<3, 1>(0, columnIndex) = localRotation.col(1).cross(preToEndTranslation);
					jacobianMatrix.block<3, 1>(3, columnIndex) = localRotation.col(1);
					columnIndex++;
				}
				double yawAngle = _calibration->getParameterValue(FixedFrameCalibration::PARAMETER_YAW);
				localRotation = localRotation * Eigen::AngleAxisd(yawAngle, Eigen::Vector3d::UnitX());
				if (_calibration->isParameterEnabled(FixedFrameCalibration::PARAMETER_YAW)) {
					jacobianMatrix.block<3, 1>(0, columnIndex) = localRotation.col(0).cross(preToEndTranslation);
					jacobianMatrix.block<3, 1>(3, columnIndex) = localRotation.col(0);
				}

				return jacobianMatrix;
		}

		//void FixedFrameJacobian::doTakeStep(const Eigen::VectorXd& step) {
		//	const rw::math::Transform3D<> correctionTransform = _calibration->getCorrectionTransform();
		//	const rw::math::Transform3D<> stepTransform = rw::math::Transform3D<>(
		//		rw::math::Vector3D<>(step(0), step(1), step(2)),
		//		rw::math::RPY<>(step(3), step(4), step(5))
		//		);
		//	const rw::math::Transform3D<> correctedCorrectionTransform = _calibration->isPostCorrection() ? correctionTransform * stepTransform : stepTransform * correctionTransform;
		//	_calibration->setCorrectionTransform(correctedCorrectionTransform);
		//}

	}
}
