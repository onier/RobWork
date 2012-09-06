/*
 * SerialDevicePoseMeasurement.cpp
 *
 *  Created on: Apr 14, 2012
 *      Author: bing
 */

#include "SerialDevicePoseMeasurement.hpp"

namespace rwlibs {
namespace calibration {

SerialDevicePoseMeasurement::SerialDevicePoseMeasurement(const rw::math::Q& state, const Pose6D<double>& pose, const Eigen::Matrix<double, 6, 6>& covariance) :
	_q(state), _pose(pose), _covariance(covariance) {

}

SerialDevicePoseMeasurement::SerialDevicePoseMeasurement(QDomElement& element) {
	if (element.tagName() == "SerialDevicePoseMeasurement") {
		QDomElement elmState = element.namedItem("Q").toElement();
		QStringList txtStateSplitted = elmState.text().split(" ");
		int stateSize = txtStateSplitted.count();
		if (stateSize > 0) {
			_q = rw::math::Q(stateSize);
			for (int variableNo = 0; variableNo < stateSize; variableNo++)
				_q[variableNo] = txtStateSplitted[variableNo].toDouble();
		}

		QDomElement elmPose = element.namedItem("Pose").toElement();
		QStringList txtPoseSplitted = elmPose.text().split(" ");
		if (txtPoseSplitted.size() == 6)
			_pose = Pose6D<double> (txtPoseSplitted[0].toDouble(), txtPoseSplitted[1].toDouble(), txtPoseSplitted[2].toDouble(), txtPoseSplitted[3].toDouble(),
					txtPoseSplitted[4].toDouble(), txtPoseSplitted[5].toDouble());

		QDomElement elmCovarianceMatrix = element.namedItem("CovarianceMatrix").toElement();
		QStringList txtCovarianceMatrixSplitted = elmCovarianceMatrix.text().split(" ");
		if (txtCovarianceMatrixSplitted.size() == 6 * 6)
			for (int rowNo = 0; rowNo < 6; rowNo++)
				for (int colNo = 0; colNo < 6; colNo++)
					_covariance(rowNo, colNo) = txtCovarianceMatrixSplitted[rowNo * 6 + colNo].toDouble();
	}
}

SerialDevicePoseMeasurement::~SerialDevicePoseMeasurement() {

}

rw::math::Q SerialDevicePoseMeasurement::getQ() const {
	return _q;
}

Pose6D<double> SerialDevicePoseMeasurement::getPose() const {
	return _pose;
}

Eigen::Matrix<double, 6, 6> SerialDevicePoseMeasurement::getCovariance() const {
	return _covariance;
}

QDomElement SerialDevicePoseMeasurement::toXml(QDomDocument& document) const {
	QDomElement elmMeasurement = document.createElement("SerialDevicePoseMeasurement");

	QDomElement elmState = document.createElement("Q");
	QString stateTxt;
	for (unsigned int variableNo = 0; variableNo < _q.size(); variableNo++)
		stateTxt.append(QString(" %1").arg(_q[variableNo], 0, 'g', 16));
	elmState.appendChild(document.createTextNode(stateTxt.trimmed()));
	elmMeasurement.appendChild(elmState);

	QDomElement elmPose = document.createElement("Pose");
	QString poseTxt;
	for (int variableNo = 0; variableNo < 6; variableNo++)
		poseTxt.append(QString(" %1").arg(_pose(variableNo), 0, 'g', 16));
	elmPose.appendChild(document.createTextNode(poseTxt.trimmed()));
	elmMeasurement.appendChild(elmPose);

	QDomElement elmCovariance = document.createElement("CovarianceMatrix");
	QString txtCovariance;
	for (int rowNo = 0; rowNo < _covariance.rows(); rowNo++)
		for (int columnNo = 0; columnNo < _covariance.cols(); columnNo++)
			txtCovariance.append(QString(" %1").arg(_covariance(rowNo, columnNo), 0, 'g', 16));
	elmCovariance.appendChild(document.createTextNode(txtCovariance.trimmed()));
	elmMeasurement.appendChild(elmCovariance);

	return elmMeasurement;
}

}
}
