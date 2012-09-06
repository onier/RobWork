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

FixedFrameCalibration::FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPreCorrection, const Eigen::Affine3d& correction) :
		_isEnabled(true), _isApplied(false), _frame(frame), _isPreCorrection(isPreCorrection), _correction(correction) {

}

FixedFrameCalibration::~FixedFrameCalibration() {

}

bool FixedFrameCalibration::isEnabled() const {
	return _isEnabled;
}

void FixedFrameCalibration::apply() {
	if (!_isEnabled)
		RW_THROW("Not enabled.");
	if (_isApplied)
		RW_THROW("Already applied.");

	Eigen::Affine3d correctedBaseTransform =
			_isPreCorrection ? Eigen::Affine3d(_frame->getFixedTransform()) * _correction : _correction * _frame->getFixedTransform();
	_frame->setTransform(correctedBaseTransform);

	_isApplied = true;
}

void FixedFrameCalibration::revert() {
	if (!_isEnabled)
		RW_THROW("Not enabled.");
	if (!_isApplied)
		RW_THROW("Not applied.");

	Eigen::Affine3d correctedBaseTransform =
			_isPreCorrection ?
					Eigen::Affine3d(_frame->getFixedTransform()) * _correction.inverse() : _correction.inverse() * _frame->getFixedTransform();
	_frame->setTransform(correctedBaseTransform);

	_isApplied = false;
}

void FixedFrameCalibration::correct(rw::kinematics::State& state) {

}

bool FixedFrameCalibration::isApplied() const {
	return _isApplied;
}

rw::kinematics::FixedFrame::Ptr FixedFrameCalibration::getFrame() const {
	return _frame;
}

bool FixedFrameCalibration::isPreCorrection() const {
	return _isPreCorrection;
}

Eigen::Affine3d FixedFrameCalibration::getCorrection() const {
	return _correction;
}

void FixedFrameCalibration::correct(const Eigen::Affine3d& correction) {
	_correction = _isPreCorrection ? _correction * correction : correction * _correction;
	if (_isEnabled && _isApplied) {
		Eigen::Affine3d correctedBaseTransform =
				_isPreCorrection ? Eigen::Affine3d(_frame->getFixedTransform()) * correction : correction * _frame->getFixedTransform();
		_frame->setTransform(correctedBaseTransform);
	}
}

QDomElement FixedFrameCalibration::toXml(QDomDocument& document) {
	QDomElement baseFrameCorrection = document.createElement("FrameCalibration");
	baseFrameCorrection.setAttribute("frame", QString::fromStdString(_frame->getName()));
	baseFrameCorrection.setAttribute("isPreCorrection", QString::number(_isPreCorrection));
	QDomElement baseFrameCorrectionTransform = document.createElement("Transform");
	QString baseTransformTxt;
	for (int rowNo = 0; rowNo < 3; rowNo++)
		for (int colNo = 0; colNo < 4; colNo++)
			baseTransformTxt.append(QString(" %1").arg(_correction(rowNo, colNo), 0, 'g', 16));
	baseFrameCorrectionTransform.appendChild(document.createTextNode(baseTransformTxt.trimmed()));
	baseFrameCorrection.appendChild(baseFrameCorrectionTransform);
	return baseFrameCorrection;
}

FixedFrameCalibration::Ptr FixedFrameCalibration::fromXml(const QDomElement& element, rw::kinematics::StateStructure::Ptr stateStructure) {
	if (!element.hasAttribute("frame"))
		RW_THROW( QString("\"frame\" attribute missing.").toStdString());
	QString frameName = element.attribute("frame");
	rw::kinematics::FixedFrame::Ptr frame = rw::kinematics::Frame::Ptr(stateStructure->findFrame(frameName.toStdString())).cast<rw::kinematics::FixedFrame>();
	if (frame.isNull())
		RW_THROW("Frame not found.");

	bool isPreCorrection = element.attribute("isPreCorrection").toInt();

	QDomElement transformNode = element.namedItem("Transform").toElement();
	QStringList txtTransformSplitted = transformNode.text().trimmed().split(" ");
	if (txtTransformSplitted.count() != 12)
		RW_THROW( QString("Transform has wrong size (12 numbers).").toStdString());
	Eigen::Affine3d transform;
	for (int rowNo = 0; rowNo < 3; rowNo++)
		for (int colNo = 0; colNo < 4; colNo++)
			transform(rowNo, colNo) = txtTransformSplitted[4 * rowNo + colNo].toDouble();

	return rw::common::ownedPtr(new FixedFrameCalibration(frame, isPreCorrection, transform));
}

}
}
