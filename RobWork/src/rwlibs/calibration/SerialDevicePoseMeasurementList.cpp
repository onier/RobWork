/*
 * SerialDevicePoseMeasurementList.cpp
 *
 *  Created on: May 22, 2012
 *      Author: bing
 */

#include "SerialDevicePoseMeasurementList.hpp"

#include <rw/common.hpp>
#include <QtCore>
#include <QtXml/qdom.h>

namespace rwlibs {
namespace calibration {

void SerialDevicePoseMeasurementList::save(const std::vector<SerialDevicePoseMeasurement::Ptr>& measurements, std::string fileName) {
	QFile file(QString::fromStdString(fileName));
	file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate);

	QDomDocument document("SerialDevicePoseMeasurements");

	QDomElement elmRoot = document.createElement("SerialDevicePoseMeasurements");
	document.appendChild(elmRoot);

	for (std::vector<SerialDevicePoseMeasurement::Ptr>::const_iterator it = measurements.begin(); it != measurements.end(); ++it) {
		SerialDevicePoseMeasurement::Ptr measurement = (*it);

		QDomElement elmMeasurement = document.createElement("SerialDevicePoseMeasurement");

		QDomElement elmState = document.createElement("Q");
		QString stateTxt;
		rw::math::Q q = measurement->getQ();
		for (unsigned int variableNo = 0; variableNo < q.size(); variableNo++)
			stateTxt.append(QString(" %1").arg(q[variableNo], 0, 'g', 16));
		elmState.appendChild(document.createTextNode(stateTxt.trimmed()));
		elmMeasurement.appendChild(elmState);

		QDomElement elmPose = document.createElement("Pose");
		QString poseTxt;
		rwlibs::calibration::Pose6D<double> pose = measurement->getPose();
		for (int variableNo = 0; variableNo < 6; variableNo++)
			poseTxt.append(QString(" %1").arg(pose(variableNo), 0, 'g', 16));
		elmPose.appendChild(document.createTextNode(poseTxt.trimmed()));
		elmMeasurement.appendChild(elmPose);

		QDomElement elmCovariance = document.createElement("CovarianceMatrix");
		QString txtCovariance;
		Eigen::Matrix<double, 6, 6> covariance = measurement->getCovariance();
		for (int rowNo = 0; rowNo < covariance.rows(); rowNo++)
			for (int columnNo = 0; columnNo < covariance.cols(); columnNo++)
				txtCovariance.append(QString(" %1").arg(covariance(rowNo, columnNo), 0, 'g', 16));
		elmCovariance.appendChild(document.createTextNode(txtCovariance.trimmed()));
		elmMeasurement.appendChild(elmCovariance);

		elmRoot.appendChild(elmMeasurement);
	}

	QTextStream textStream(&file);
	textStream.setRealNumberPrecision(16);
	textStream << document.toString();

	file.close();
}

std::vector<SerialDevicePoseMeasurement::Ptr> SerialDevicePoseMeasurementList::load(std::string fileName) {
	std::vector<SerialDevicePoseMeasurement::Ptr> measurements;

	QFile file(QString::fromStdString(fileName));
	file.open(QIODevice::ReadOnly | QIODevice::Text | QIODevice::Truncate);

	QDomDocument document("SerialDevicePoseMeasurements");

	if (!document.setContent(&file))
		RW_THROW("Content not set.");

	QDomElement elmRoot = document.documentElement();
	if (elmRoot.tagName() != "SerialDevicePoseMeasurements")
		RW_THROW("Root element not found.");

	QDomNode node = elmRoot.firstChild();
	while (!node.isNull()) {
		QDomElement element = node.toElement();
		if (!element.isNull() && element.tagName() == "SerialDevicePoseMeasurement") {
				QDomElement elmState = element.namedItem("Q").toElement();
				QStringList txtStateSplitted = elmState.text().split(" ");
				int stateSize = txtStateSplitted.count();
				if (stateSize <= 0)
					RW_THROW("Q not parsed correctly.");
				rw::math::Q q = rw::math::Q(stateSize);
				for (int variableNo = 0; variableNo < stateSize; variableNo++)
					q[variableNo] = txtStateSplitted[variableNo].toDouble();

				QDomElement elmPose = element.namedItem("Pose").toElement();
				QStringList txtPoseSplitted = elmPose.text().split(" ");
				if (txtPoseSplitted.size() != 6)
					RW_THROW("Pose not parsed correctly.");
				rwlibs::calibration::Pose6D<double> pose = Pose6D<double>(txtPoseSplitted[0].toDouble(), txtPoseSplitted[1].toDouble(),
						txtPoseSplitted[2].toDouble(), txtPoseSplitted[3].toDouble(), txtPoseSplitted[4].toDouble(), txtPoseSplitted[5].toDouble());

				QDomElement elmCovarianceMatrix = element.namedItem("CovarianceMatrix").toElement();
				QStringList txtCovarianceMatrixSplitted = elmCovarianceMatrix.text().split(" ");
				if (txtCovarianceMatrixSplitted.size() != 6 * 6)
					RW_THROW("Covariance matrix not parsed correctly.");
				Eigen::Matrix<double, 6, 6> covariance;
				for (int rowNo = 0; rowNo < 6; rowNo++)
					for (int colNo = 0; colNo < 6; colNo++)
						covariance(rowNo, colNo) = txtCovarianceMatrixSplitted[rowNo * 6 + colNo].toDouble();

				measurements.push_back(rw::common::ownedPtr(new SerialDevicePoseMeasurement(q, pose, covariance)));
		}

		node = node.nextSibling();
	}

	return measurements;
}

}
}
