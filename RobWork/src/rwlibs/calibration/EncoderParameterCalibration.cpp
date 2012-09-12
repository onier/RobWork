/*
 * EncoderParameterCalibration.cpp
 *
 *  Created on: Aug 28, 2012
 *      Author: bing
 */

#include "EncoderParameterCalibration.hpp"

#include <sandbox/calibration/EncoderDecentralization.hpp>

namespace rwlibs {
namespace calibration {

EncoderParameterCalibration::EncoderParameterCalibration(rw::models::JointDevice::Ptr jointDevice, rw::models::Joint::Ptr joint,
		const Eigen::Vector2d& correction) :
		_jointDevice(jointDevice), _joint(joint), _correction(correction) {
	// Find joint number.
	const std::vector<rw::models::Joint*> joints = jointDevice->getJoints();
	_jointNo = std::find(joints.begin(), joints.end(), joint.get()) - joints.begin();
}

EncoderParameterCalibration::~EncoderParameterCalibration() {

}

rw::models::Joint::Ptr EncoderParameterCalibration::getJoint() const {
	return _joint;
}

Eigen::Vector2d EncoderParameterCalibration::getCorrection() const {
	return _correction;
}

void EncoderParameterCalibration::correct(const Eigen::Vector2d& correction) {
	_correction = _correction + correction;
}

QDomElement EncoderParameterCalibration::toXml(QDomDocument& document) {
	QDomElement element = document.createElement("EDParameterCalibration");

	element.setAttribute("joint", QString::fromStdString(_joint->getName()));

	element.setAttribute("tau", QString("%1").arg(_correction(0), 0, 'g', 16));
	element.setAttribute("sigma", QString("%1").arg(_correction(1), 0, 'g', 16));

	return element;
}

EncoderParameterCalibration::Ptr EncoderParameterCalibration::fromXml(const QDomElement& element, rw::kinematics::StateStructure::Ptr stateStructure, rw::models::JointDevice::Ptr jointDevice) {
	if (!element.hasAttribute("joint"))
		RW_THROW("\"joint\" attribute missing.");
	QString jointName = element.attribute("joint");

	rw::models::Joint::Ptr joint = (rw::models::Joint*) stateStructure->findFrame(jointName.toStdString());
	if (joint.isNull())
		RW_THROW(QString("Joint with name \"%1\" not found.").arg(jointName).toStdString());

	if (!element.hasAttribute("tau"))
		RW_THROW(QString("Joint \"%1\" needs \"tau\" attribute.").arg(jointName).toStdString());
	double tau = element.attribute("tau").toDouble();

	if (!element.hasAttribute("sigma"))
		RW_THROW(QString("Joint \"%1\" needs \"sigma\" attribute.").arg(jointName).toStdString());
	double sigma = element.attribute("sigma").toDouble();

	return rw::common::ownedPtr(new EncoderParameterCalibration(jointDevice, joint, Eigen::Vector2d(tau, sigma)));
}

void EncoderParameterCalibration::doApply() {

}

void EncoderParameterCalibration::doRevert() {

}

void EncoderParameterCalibration::doCorrect(rw::kinematics::State& state) {
	rw::math::Q q = _jointDevice->getQ(state);
	// Implemented directly, so that sandbox is not required.
//	q[_jointNo] = rw::models::EncoderDecentralization::calcRealAngle(q[_jointNo], _correction(0), _correction(1));
	q[_jointNo] = q[_jointNo] - (_correction(1) * cos(q[_jointNo]) + _correction(0) * sin(q[_jointNo]));
	_jointDevice->setQ(q, state);
}

}
}
