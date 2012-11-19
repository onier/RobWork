/*
 * XmlMeasurementFile.cpp
 *
 *  Created on: May 22, 2012
 *      Author: bing
 */

#include "XmlMeasurementFile.hpp"

#include <rw/common.hpp>

#include <QtCore>
#include <QtXml/qdom.h>

namespace rwlibs {
namespace calibration {

QDomElement convertMeasurementToDomElement(QDomDocument& document, SerialDevicePoseMeasurement::Ptr measurement) {
	QDomElement elmMeasurement = document.createElement("SerialDevicePoseMeasurement");

	QDomElement elmState = document.createElement("Q");
	QString stateTxt;
	rw::math::Q q = measurement->getQ();
	for (unsigned int variableNo = 0; variableNo < q.size(); variableNo++)
		stateTxt.append(QString(" %1").arg(q[variableNo], 0, 'g', 16));
	elmState.appendChild(document.createTextNode(stateTxt.simplified()));
	elmMeasurement.appendChild(elmState);

	QDomElement elmTransform = document.createElement("Transform");
	QString transformTxt;
	rw::math::Transform3D<> transform = measurement->getTransform();
	for (int rowIndex = 0; rowIndex < 3; rowIndex++)
		for (int columnIndex = 0; columnIndex < 4; columnIndex++)
				transformTxt.append(QString(" %1").arg(transform(rowIndex, columnIndex), 0, 'g', 16));
	elmTransform.appendChild(document.createTextNode(transformTxt.simplified()));
	elmMeasurement.appendChild(elmTransform);

	if (measurement->hasCovariance()) {
		QDomElement elmCovariance = document.createElement("CovarianceMatrix");
		QString txtCovariance;
		Eigen::Matrix<double, 6, 6> covariance = measurement->getCovariance();
		for (int rowIndex = 0; rowIndex < covariance.rows(); rowIndex++)
			for (int columnIndex = 0; columnIndex < covariance.cols(); columnIndex++)
				txtCovariance.append(QString(" %1").arg(covariance(rowIndex, columnIndex), 0, 'g', 16));
		elmCovariance.appendChild(document.createTextNode(txtCovariance.simplified()));
		elmMeasurement.appendChild(elmCovariance);
	}

	return elmMeasurement;
}

SerialDevicePoseMeasurement::Ptr convertDomElementToMeasurement(const QDomElement& element) {
	if (element.tagName() != "SerialDevicePoseMeasurement")
		return NULL;

	QDomElement elmState = element.namedItem("Q").toElement();
	QStringList txtStateSplitted = elmState.text().simplified().split(" ");
	int stateSize = txtStateSplitted.count();
	if (stateSize <= 0)
		RW_THROW("Q not parsed correctly.");
	rw::math::Q q = rw::math::Q(stateSize);
	for (int variableNo = 0; variableNo < stateSize; variableNo++)
		q[variableNo] = txtStateSplitted[variableNo].toDouble();

	QDomElement elmTransform = element.namedItem("Transform").toElement();
	QStringList txtTransformSplitted = elmTransform.text().simplified().split(" ");
	if (txtTransformSplitted.size() != 3 * 4)
		RW_THROW("Transform not parsed correctly.");
	rw::math::Transform3D<> transform;
	for (int rowIndex = 0; rowIndex < 3; rowIndex++)
		for (int columnIndex = 0; columnIndex < 4; columnIndex++)
			transform(rowIndex, columnIndex) = txtTransformSplitted[rowIndex * 4 + columnIndex].toDouble();

	Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
	if (!element.namedItem("CovarianceMatrix").isNull()) {
		QDomElement elmCovarianceMatrix = element.namedItem("CovarianceMatrix").toElement();
		QStringList txtCovarianceMatrixSplitted = elmCovarianceMatrix.text().simplified().split(" ");
		if (txtCovarianceMatrixSplitted.size() != 6 * 6)
			RW_THROW("Covariance matrix not parsed correctly.");
		for (int rowIndex = 0; rowIndex < 6; rowIndex++)
			for (int columnIndex = 0; columnIndex < 6; columnIndex++)
				covariance(rowIndex, columnIndex) = txtCovarianceMatrixSplitted[rowIndex * 6 + columnIndex].toDouble();
	}

	return rw::common::ownedPtr(new SerialDevicePoseMeasurement(q, transform, covariance));
}

void XmlMeasurementFile::save(const std::vector<SerialDevicePoseMeasurement::Ptr>& measurements, std::string fileName) {
	QDomDocument document("SerialDevicePoseMeasurements");

	QDomElement elmRoot = document.createElement("SerialDevicePoseMeasurements");
	document.appendChild(elmRoot);

	for (std::vector<SerialDevicePoseMeasurement::Ptr>::const_iterator it = measurements.begin(); it != measurements.end(); ++it) {
		SerialDevicePoseMeasurement::Ptr measurement = (*it);
		QDomElement elmMeasurement = convertMeasurementToDomElement(document, measurement);
		elmRoot.appendChild(elmMeasurement);
	}

	QFile file(QString::fromStdString(fileName));
	file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate);
	QTextStream textStream(&file);
	textStream.setRealNumberPrecision(16);
	textStream << document.toString();
}

std::vector<SerialDevicePoseMeasurement::Ptr> XmlMeasurementFile::load(std::string fileName) {
	QFile file(QString::fromStdString(fileName));
	file.open(QIODevice::ReadOnly | QIODevice::Text | QIODevice::Truncate);

	QDomDocument document("SerialDevicePoseMeasurements");
	if (!document.setContent(&file))
		RW_THROW("Parsing of measurement file failed.");

	QDomElement elmRoot = document.documentElement();
	if (elmRoot.tagName() != "SerialDevicePoseMeasurements")
		RW_THROW("No measurements found in measurement file.");

	QDomNode node = elmRoot.firstChild();
	std::vector<SerialDevicePoseMeasurement::Ptr> measurements;
	while (!node.isNull()) {
		QDomElement domElement = node.toElement();
		if (!domElement.isNull()) {
			SerialDevicePoseMeasurement::Ptr measurement = convertDomElementToMeasurement(domElement);
			measurements.push_back(measurement);
		}
		node = node.nextSibling();
	}

	return measurements;
}

}
}
