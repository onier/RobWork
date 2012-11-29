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
		RW_THROW("\"frame\" attribute missing.");
	std::string frameName = element.attribute("frame").toStdString();
	rw::kinematics::Frame* frame = _stateStructure->findFrame(frameName);
	rw::kinematics::FixedFrame::Ptr fixedFrame = rw::kinematics::Frame::Ptr(frame).cast<rw::kinematics::FixedFrame>();
	if (fixedFrame.isNull())
		RW_THROW("Frame \"" << frameName << "\" not found.");
	
	if (!element.hasAttribute("isPostCorrection"))
		RW_THROW("\"isPostCorrection\" attribute missing.");
	bool isPostCorrection = element.attribute("isPostCorrection").toInt();

	QDomElement transformElement = element.namedItem("Transform").toElement();
	if (transformElement.isNull())
		RW_THROW("\"Transform\" element not found");
	QStringList txtTransformSplitted = transformElement.text().simplified().split(" ");
	txtTransformSplitted.removeAll(" ");
	if (txtTransformSplitted.count() != 12)
		RW_THROW("Transform has wrong size (12 numbers).");
	Eigen::Affine3d transform;
	for (int rowIndex = 0; rowIndex < 3; rowIndex++)
		for (int columnIndex = 0; columnIndex < 4; columnIndex++)
			transform(rowIndex, columnIndex) = txtTransformSplitted[4 * rowIndex + columnIndex].toDouble();

	return rw::common::ownedPtr(new FixedFrameCalibration(fixedFrame, isPostCorrection, transform));
}

template<>
DHParameterCalibration::Ptr ElementReader::readElement<DHParameterCalibration::Ptr>(const QDomElement& element) {
	if (!element.hasAttribute("joint"))
		RW_THROW("\"joint\" attribute missing.");
	std::string jointName = element.attribute("joint").toStdString();

	rw::models::Joint::Ptr joint = (rw::models::Joint*) _stateStructure->findFrame(jointName);
	if (joint.isNull())
		RW_THROW("Joint \"" << jointName << "\" not found.");

	DHParameterCalibration::Ptr calibration = rw::common::ownedPtr(new DHParameterCalibration(joint));

	if (!element.hasAttribute("a"))
		calibration->setParameterEnabled(DHParameterCalibration::PARAMETER_A, false);
	else
		calibration->setParameterValue(DHParameterCalibration::PARAMETER_A, element.attribute("a").toDouble());

	if (!element.hasAttribute("b"))
		calibration->setParameterEnabled(DHParameterCalibration::PARAMETER_B, false);
	else
		calibration->setParameterValue(DHParameterCalibration::PARAMETER_B, element.attribute("b").toDouble());

	if (!element.hasAttribute("d"))
		calibration->setParameterEnabled(DHParameterCalibration::PARAMETER_D, false);
	else
		calibration->setParameterValue(DHParameterCalibration::PARAMETER_D, element.attribute("d").toDouble());

	if (!element.hasAttribute("alpha"))
		calibration->setParameterEnabled(DHParameterCalibration::PARAMETER_ALPHA, false);
	else
		calibration->setParameterValue(DHParameterCalibration::PARAMETER_ALPHA, element.attribute("alpha").toDouble());

	if (!element.hasAttribute("beta"))
		calibration->setParameterEnabled(DHParameterCalibration::PARAMETER_BETA, false);
	else
		calibration->setParameterValue(DHParameterCalibration::PARAMETER_BETA, element.attribute("beta").toDouble());

	if (!element.hasAttribute("theta"))
		calibration->setParameterEnabled(DHParameterCalibration::PARAMETER_THETA, false);
	else
		calibration->setParameterValue(DHParameterCalibration::PARAMETER_THETA, element.attribute("theta").toDouble());

	return calibration;
}

SerialDeviceCalibration::Ptr XmlCalibrationLoader::load(rw::kinematics::StateStructure::Ptr stateStructure,
		rw::models::SerialDevice::Ptr device, std::string fileName) {
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
		endCalibration = elementReader.readElement<FixedFrameCalibration::Ptr>(nodeEnd.childNodes().at(0).toElement());

	// Load DH calibrations
	CompositeCalibration<DHParameterCalibration>::Ptr dhCalibrations = rw::common::ownedPtr(new CompositeCalibration<DHParameterCalibration>());
	QDomNode nodeDH = elmRoot.namedItem("DHParameterCalibrations");
	if (!nodeDH.isNull()) {
		QDomNodeList nodes = nodeDH.childNodes();
		for (int nodeIndex = 0; nodeIndex < nodes.size(); nodeIndex++)
			dhCalibrations->addCalibration(elementReader.readElement<DHParameterCalibration::Ptr>(nodes.at(nodeIndex).toElement()));
	}

	SerialDeviceCalibration::Ptr calibration = rw::common::ownedPtr(
			new SerialDeviceCalibration(device, baseCalibration, endCalibration, dhCalibrations));

	return calibration;
}

}
}
