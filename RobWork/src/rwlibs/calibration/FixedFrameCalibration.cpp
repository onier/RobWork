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
			CalibrationBase(6), _frame(frame), _isPostCorrection(false) {
		}

		FixedFrameCalibration::FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPostCorrection) :
			CalibrationBase(6), _frame(frame), _isPostCorrection(isPostCorrection) {
		}

		FixedFrameCalibration::FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPostCorrection, const rw::math::Transform3D<>& correctionTransform) :
			CalibrationBase(6), _frame(frame), _isPostCorrection(isPostCorrection) {
			setCorrectionTransform(correctionTransform);
		}

		FixedFrameCalibration::~FixedFrameCalibration() {

		}

		rw::kinematics::FixedFrame::Ptr FixedFrameCalibration::getFrame() const {
			return _frame;
		}

		bool FixedFrameCalibration::isPostCorrection() const {
			return _isPostCorrection;
		}

		rw::math::Transform3D<> FixedFrameCalibration::getCorrectionTransform() const {
			rw::math::Vector3D<> translation(
				isParameterEnabled(PARAMETER_X) ? getParameterValue(PARAMETER_X) : 0.0,
				isParameterEnabled(PARAMETER_Y) ? getParameterValue(PARAMETER_Y) : 0.0,
				isParameterEnabled(PARAMETER_Z) ? getParameterValue(PARAMETER_Z) : 0.0
				);
			rw::math::RPY<> rpy(
				isParameterEnabled(PARAMETER_ROLL) ? getParameterValue(PARAMETER_ROLL) : 0.0,
				isParameterEnabled(PARAMETER_PITCH) ? getParameterValue(PARAMETER_PITCH) : 0.0,
				isParameterEnabled(PARAMETER_YAW) ? getParameterValue(PARAMETER_YAW) : 0.0
				);
			return rw::math::Transform3D<>(translation, rpy.toRotation3D());
		}

		void FixedFrameCalibration::setCorrectionTransform(const rw::math::Transform3D<>& transform) {
			// HACK: Fix hack.
			setParameterValue(PARAMETER_X, transform.P()(0));
			setParameterValue(PARAMETER_Y, transform.P()(1));
			setParameterValue(PARAMETER_Z, transform.P()(2));
			rw::math::RPY<> rpy(transform.R());
			setParameterValue(PARAMETER_ROLL, rpy(0));
			setParameterValue(PARAMETER_PITCH, rpy(1));
			setParameterValue(PARAMETER_YAW, rpy(2));
		}

		void FixedFrameCalibration::doApply() {
			RW_ASSERT(!_frame.isNull());
			_originalTransform = _frame->getFixedTransform();
			const rw::math::Transform3D<> correctionTransform = getCorrectionTransform();
			const rw::math::Transform3D<> correctedTransform = _isPostCorrection ? (_originalTransform * correctionTransform) : (correctionTransform * _originalTransform);
			_frame->setTransform(correctedTransform);
		}

		void FixedFrameCalibration::doRevert() {
			RW_ASSERT(!_frame.isNull());
			_frame->setTransform(_originalTransform);
		}
	}
}
