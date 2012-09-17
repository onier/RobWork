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
		_frame(frame), _isPreCorrection(isPreCorrection), _correction(correction), _enabledParameters(Eigen::Matrix<int, 6, 1>::Ones()) {

}

FixedFrameCalibration::~FixedFrameCalibration() {

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

void FixedFrameCalibration::setEnabledParameters(bool x, bool y, bool z, bool roll, bool pitch, bool yaw) {
	_enabledParameters << x, y, z, roll, pitch, yaw;
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

void FixedFrameCalibration::doApply() {
	Eigen::Affine3d newTransform =
			_isPreCorrection ? Eigen::Affine3d(_frame->getFixedTransform()) * _correction : _correction * _frame->getFixedTransform();
	_frame->setTransform(newTransform);
}

void FixedFrameCalibration::doRevert() {
	Eigen::Affine3d newTransform =
			_isPreCorrection ?
					Eigen::Affine3d(_frame->getFixedTransform()) * _correction.inverse() : _correction.inverse() * _frame->getFixedTransform();
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
	_correction = _isPreCorrection ? _correction * stepTransform : stepTransform * _correction;
	if (isApplied()) {
		Eigen::Affine3d correctedBaseTransform =
				_isPreCorrection ? Eigen::Affine3d(_frame->getFixedTransform()) * stepTransform : stepTransform * _frame->getFixedTransform();
		_frame->setTransform(correctedBaseTransform);
	}
}

}
}
