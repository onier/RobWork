/*
 * SerialDevicePoseMeasurementList.cpp
 *
 *  Created on: May 22, 2012
 *      Author: bing
 */

#include "SerialDevicePoseMeasurementList.hpp"

#include <QtXml/qdom.h>

namespace rwlibs {
namespace calibration {

void SerialDevicePoseMeasurementList::save(std::string fileName) {
	QFile file(QString::fromStdString(fileName));
	file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate);

	QDomDocument document("SerialDevicePoseMeasurements");

	QDomElement elmRoot = document.createElement("SerialDevicePoseMeasurements");
	document.appendChild(elmRoot);

	for (int measurementNo = 0; measurementNo < size(); measurementNo++)
		elmRoot.appendChild(at(measurementNo).toXml(document));

	QTextStream textStream(&file);
	textStream.setRealNumberPrecision(16);
	textStream << document.toString();

	file.close();
}

SerialDevicePoseMeasurementList SerialDevicePoseMeasurementList::load(std::string fileName) {
	SerialDevicePoseMeasurementList list;

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
		if (!element.isNull())
			list.append(SerialDevicePoseMeasurement(element));

		node = node.nextSibling();
	}

	return list;
}

}
}
