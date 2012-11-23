/*
 * XmlCalibrationSaver.cpp
 *
 *  Created on: Sep 19, 2012
 *      Author: bing
 */

#include "XmlCalibrationSaver.hpp"

#include <QtCore>
#include <QtXml/qdom.h>

namespace rwlibs {
namespace calibration {

class ElementCreator {
public:
	ElementCreator(QDomDocument* document) :
			_document(document) {

	}

	template<class T>
	QDomElement createElement(T object);

private:
	QDomDocument* _document;
};

template<>
QDomElement ElementCreator::createElement<FixedFrameCalibration::Ptr>(FixedFrameCalibration::Ptr calibration) {
	QDomElement element = _document->createElement("FixedFrameCalibration");

	element.setAttribute("frame", QString::fromStdString(calibration->getFrame()->getName()));
	element.setAttribute("isPostCorrection", QString::number(calibration->isPostCorrection()));

	QDomElement elmTransform = _document->createElement("Transform");

	QString baseTransformTxt;
	Eigen::Affine3d correction = calibration->getCorrectionTransform();
	for (int rowIndex = 0; rowIndex < 3; rowIndex++)
		for (int colIndex = 0; colIndex < 4; colIndex++)
			baseTransformTxt.append(QString(" %1").arg(correction(rowIndex, colIndex), 0, 'g', 16));
	elmTransform.appendChild(_document->createTextNode(baseTransformTxt.trimmed()));
	element.appendChild(elmTransform);

	return element;
}

template<>
QDomElement ElementCreator::createElement<DHParameterCalibration::Ptr>(DHParameterCalibration::Ptr calibration) {
	QDomElement element = _document->createElement("DHParameterCalibration");

	element.setAttribute("joint", QString::fromStdString(calibration->getJoint()->getName()));

	Eigen::Vector4d correction = calibration->getCorrection();
	element.setAttribute("a", QString("%1").arg(correction(0), 0, 'g', 16));
	element.setAttribute("length", QString("%1").arg(correction(1), 0, 'g', 16));
	element.setAttribute("alpha", QString("%1").arg(correction(2), 0, 'g', 16));
	element.setAttribute("angle", QString("%1").arg(correction(3), 0, 'g', 16));

	return element;
}

QDomDocument createDOMDocument(SerialDeviceCalibration::Ptr calibration) {
	QDomDocument document("SerialDeviceCalibration");

	QDomElement elmRoot = document.createElement("SerialDeviceCalibration");

	ElementCreator creator(&document);

	// Save base correction
	if (!calibration->getBaseCalibration().isNull()) {
		QDomElement elmBase = document.createElement("BaseFrameCalibration");
		elmBase.appendChild(creator.createElement<FixedFrameCalibration::Ptr>(calibration->getBaseCalibration()));
		elmRoot.appendChild(elmBase);
	}

	// Save end correction
	if (!calibration->getEndCalibration().isNull()) {
		QDomElement elmEnd = document.createElement("EndFrameCalibration");
		elmEnd.appendChild(creator.createElement<FixedFrameCalibration::Ptr>(calibration->getEndCalibration()));
		elmRoot.appendChild(elmEnd);
	}

	// Save dh corrections
	std::vector<DHParameterCalibration::Ptr> dhParameterCalibrations = calibration->getCompositeDHParameterCalibration()->getCalibrations();
	if (dhParameterCalibrations.size() > 0) {
		QDomElement dhCorrections = document.createElement("DHParameterCalibrations");
		for (std::vector<DHParameterCalibration::Ptr>::iterator it = dhParameterCalibrations.begin(); it != dhParameterCalibrations.end(); ++it) {
			DHParameterCalibration::Ptr dhParameterCalibration = (*it);
			dhCorrections.appendChild(creator.createElement<DHParameterCalibration::Ptr>(dhParameterCalibration));
		}
		elmRoot.appendChild(dhCorrections);
	}

	//	// Save encoder corrections
	//	std::vector<EncoderParameterCalibration::Ptr> encoderParameterCalibrations = _compositeEncoderParameterCalibration->getCalibrations();
	//	if (encoderParameterCalibrations.size() > 0) {
	//		QDomElement encoderCorrections = document.createElement("EncoderParameterCalibrations");
	//		for (std::vector<EncoderParameterCalibration::Ptr>::iterator it = encoderParameterCalibrations.begin(); it != encoderParameterCalibrations.end();
	//				++it) {
	//			EncoderParameterCalibration::Ptr encoderCalibration = (*it);
	//			encoderCorrections.appendChild(encoderCalibration->toXml(document));
	//		}
	//		elmRoot.appendChild(encoderCorrections);
	//	}

	document.appendChild(elmRoot);

	return document;
}

void XmlCalibrationSaver::save(SerialDeviceCalibration::Ptr calibration, std::string fileName) {
	QFile file(QString::fromStdString(fileName));
	file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate);

	QTextStream textStream(&file);
	textStream.setRealNumberPrecision(16);
	textStream << createDOMDocument(calibration).toString();

	file.close();
}

void XmlCalibrationSaver::save(SerialDeviceCalibration::Ptr calibration, std::ostream& ostream) {
	ostream << createDOMDocument(calibration).toString().toStdString();
}

}
}
