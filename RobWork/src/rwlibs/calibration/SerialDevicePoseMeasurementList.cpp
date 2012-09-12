/*
 * SerialDevicePoseMeasurementList.cpp
 *
 *  Created on: May 22, 2012
 *      Author: bing
 */

#include "SerialDevicePoseMeasurementList.hpp"

#include <QtCore>
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
	std::cout << "1." << std::endl;
	while (!node.isNull()) {
		QDomElement element = node.toElement();
		std::cout << "2." << std::endl;
		if (!element.isNull())
			list.push_back(SerialDevicePoseMeasurement(element));
		std::cout << "3." << std::endl;

		node = node.nextSibling();
	}
	std::cout << "4." << std::endl;

	return list;
}

}
}
