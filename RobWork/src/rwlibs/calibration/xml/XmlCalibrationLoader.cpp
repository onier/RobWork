/*
 * XmlCalibrationFile.cpp
 *
 *  Created on: Sep 18, 2012
 *      Author: bing
 */

#include "XmlCalibrationLoader.hpp"

#include <QtCore>
#include <QtXml/qdom.h>

namespace rwlibs {
namespace calibration {

class ElementReader {
public:
	ElementReader(rw::kinematics::StateStructure::Ptr stateStructure) :
			_stateStructure(stateStructure) {

	}

	template<class T>
	T readElement(const QDomElement& element);

private:
	rw::kinematics::StateStructure::Ptr _stateStructure;
};

template<>
FixedFrameCalibration::Ptr ElementReader::readElement<FixedFrameCalibration::Ptr>(const QDomElement& element) {
	if (!element.hasAttribute("frame"))
		RW_THROW( QString("\"frame\" attribute missing.").toStdString());
	QString frameName = element.attribute("frame");
	rw::kinematics::FixedFrame::Ptr frame = rw::kinematics::Frame::Ptr(_stateStructure->findFrame(frameName.toStdString())).cast<rw::kinematics::FixedFrame>();
	if (frame.isNull())
		RW_THROW("Frame not found.");

	bool isPreCorrection = element.attribute("isPreCorrection").toInt();

	QDomElement transformNode = element.namedItem("Transform").toElement();
	QStringList txtTransformSplitted = transformNode.text().simplified().split(" ");
	txtTransformSplitted.removeAll(" ");
	if (txtTransformSplitted.count() != 12)
		RW_THROW( QString("Transform has wrong size (12 numbers).").toStdString());
	Eigen::Affine3d transform;
	for (int rowIndex = 0; rowIndex < 3; rowIndex++)
		for (int columnIndex = 0; columnIndex < 4; columnIndex++)
			transform(rowIndex, columnIndex) = txtTransformSplitted[4 * rowIndex + columnIndex].toDouble();

	return rw::common::ownedPtr(new FixedFrameCalibration(frame, isPreCorrection, transform));
}

template<>
DHParameterCalibration::Ptr ElementReader::readElement<DHParameterCalibration::Ptr>(const QDomElement& element) {
	if (!element.hasAttribute("joint"))
		RW_THROW("\"joint\" attribute missing.");
	QString jointName = element.attribute("joint");

	rw::models::Joint::Ptr joint = (rw::models::Joint*) _stateStructure->findFrame(jointName.toStdString());
	if (joint.isNull())
		RW_THROW(QString("Joint \"%1\" not found.").arg(jointName).toStdString());

	if (!element.hasAttribute("a"))
		RW_THROW(QString("Joint \"%1\" needs \"a\" attribute.").arg(jointName).toStdString());
	double a = element.attribute("a").toDouble();

	if (!element.hasAttribute("length"))
		RW_THROW(QString("Joint \"%1\" needs \"length\" attribute.").arg(jointName).toStdString());
	double length = element.attribute("length").toDouble();

	if (!element.hasAttribute("alpha"))
		RW_THROW(QString("Joint \"%1\" needs \"alpha\" attribute.").arg(jointName).toStdString());
	double alpha = element.attribute("alpha").toDouble();

	if (!element.hasAttribute("angle"))
		RW_THROW(QString("Joint \"%1\" needs \"angle\" attribute.").arg(jointName).toStdString());
	double angle = element.attribute("angle").toDouble();

	return rw::common::ownedPtr(
			new DHParameterCalibration(joint, Eigen::Vector4d(a, length, alpha, angle)));
}

SerialDeviceCalibration::Ptr XmlCalibrationLoader::load(std::string fileName, rw::kinematics::StateStructure::Ptr stateStructure,
		rw::models::SerialDevice::Ptr device) {
	QFile file(QString::fromStdString(fileName));
	file.open(QIODevice::ReadOnly | QIODevice::Text | QIODevice::Truncate);

	QDomDocument document("SerialDeviceCalibration");
	if (!document.setContent(&file))
		RW_THROW("Content not set.");

	QDomElement elmRoot = document.documentElement();
	if (elmRoot.tagName() != "SerialDeviceCalibration")
		RW_THROW("Element not found.");

	ElementReader elementReader(stateStructure);

	// Load base frame calibration
	FixedFrameCalibration::Ptr baseCalibration;
	QDomNode nodeBase = elmRoot.namedItem("BaseFrameCalibration");
	if (!nodeBase.isNull() && nodeBase.hasChildNodes())
		baseCalibration = elementReader.readElement<FixedFrameCalibration::Ptr>(nodeBase.childNodes().at(0).toElement());

	// Load end frame calibration
	FixedFrameCalibration::Ptr endCalibration;
	QDomNode nodeEnd = elmRoot.namedItem("EndFrameCalibration");
	if (!nodeEnd.isNull() && nodeEnd.hasChildNodes())
		endCalibration = elementReader.readElement<FixedFrameCalibration::Ptr>(nodeBase.childNodes().at(0).toElement());

	// Load DH calibrations
	CompositeCalibration<DHParameterCalibration>::Ptr dhCalibrations = rw::common::ownedPtr(new CompositeCalibration<DHParameterCalibration>());
	QDomNode nodeDH = elmRoot.namedItem("DHParameterCalibrations");
	if (!nodeDH.isNull()) {
		QDomNodeList nodes = nodeDH.childNodes();
		for (int nodeIndex = 0; nodeIndex < nodes.size(); nodeIndex++)
			dhCalibrations->add(elementReader.readElement<DHParameterCalibration::Ptr>(nodes.at(nodeIndex).toElement()));
	}

	//	// Load encoder calibrations
	//	CompositeCalibration<EncoderParameterCalibration>::Ptr encoderCalibrations = rw::common::ownedPtr(new CompositeCalibration<EncoderParameterCalibration>());
	//	QDomNode nodeEncoder = elmRoot.namedItem("EncoderParameterCalibrations");
	//	if (!nodeEncoder.isNull()) {
	//		QDomNodeList nodes = nodeEncoder.childNodes();
	//		for (int nodeNo = 0; nodeNo < nodes.size(); nodeNo++) {
	//			QDomElement element = nodes.at(nodeNo).toElement();
	//			encoderCalibrations->add(EncoderParameterCalibration::fromXml(element, stateStructure, device));
	//		}
	//	}

	SerialDeviceCalibration::Ptr calibration = rw::common::ownedPtr(
			new SerialDeviceCalibration(device, baseCalibration, endCalibration, dhCalibrations/*, encoderCalibrations*/));

	return calibration;
}

}
}
