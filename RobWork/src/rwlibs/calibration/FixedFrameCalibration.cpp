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
		_frame(frame), _isPostCorrection(false), _correctionTransform(Eigen::Affine3d::Identity()) {
}

FixedFrameCalibration::FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPostCorrection) :
		_frame(frame), _isPostCorrection(isPostCorrection), _correctionTransform(Eigen::Affine3d::Identity()) {
}

FixedFrameCalibration::FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPostCorrection, const rw::math::Transform3D<>& correctionTransform) :
		_frame(frame), _isPostCorrection(isPostCorrection), _correctionTransform(correctionTransform) {
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
	return _correctionTransform;
}

void FixedFrameCalibration::setCorrectionTransform(const rw::math::Transform3D<>& correctionTransform) {
	_correctionTransform = correctionTransform;
}

void FixedFrameCalibration::doApply() {
	RW_ASSERT(!_frame.isNull());
	_originalTransform = _frame->getFixedTransform();
	const rw::math::Transform3D<> correctedTransform = _isPostCorrection ? (_correctionTransform * _originalTransform) : (_originalTransform * _correctionTransform);
	_frame->setTransform(correctedTransform);
}

void FixedFrameCalibration::doRevert() {
	RW_ASSERT(!_frame.isNull());
	_frame->setTransform(_originalTransform);
}

void FixedFrameCalibration::doCorrectState(rw::kinematics::State& state) {

}

}
}
