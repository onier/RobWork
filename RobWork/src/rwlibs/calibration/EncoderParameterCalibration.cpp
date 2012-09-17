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
		_jointDevice(jointDevice), _joint(joint), _correction(correction), _enabledParameters(Eigen::Vector2i::Ones()) {
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

void EncoderParameterCalibration::setEnabledParameters(bool tau, bool sigma) {
	_enabledParameters << tau, sigma;
}

QDomElement EncoderParameterCalibration::toXml(QDomDocument& document) {
	QDomElement element = document.createElement("EDParameterCalibration");

	element.setAttribute("joint", QString::fromStdString(_joint->getName()));

	element.setAttribute("tau", QString("%1").arg(_correction(0), 0, 'g', 16));
	element.setAttribute("sigma", QString("%1").arg(_correction(1), 0, 'g', 16));

	return element;
}

EncoderParameterCalibration::Ptr EncoderParameterCalibration::fromXml(const QDomElement& element, rw::kinematics::StateStructure::Ptr stateStructure,
		rw::models::JointDevice::Ptr jointDevice) {
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

int EncoderParameterCalibration::doGetParameterCount() const {
	return _enabledParameters.sum();
}

Eigen::MatrixXd EncoderParameterCalibration::doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
		const rw::kinematics::State& state) {
	// Get joint value.
	const rw::math::Q q = _jointDevice->getQ(state);
	const double qi = q[_jointNo];

	// Prepare transformations.
	const Eigen::Affine3d tfmToPostJoint = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), _joint.get(), state);
	const Eigen::Affine3d tfmPostJoint = rw::kinematics::Kinematics::frameTframe(_joint.get(), measurementFrame.get(), state);
	const Eigen::Affine3d tfmToEnd = tfmToPostJoint * tfmPostJoint;
	const Eigen::Vector3d posToEnd = tfmToEnd.translation() - tfmToPostJoint.translation();
	const Eigen::Vector3d jointAxis = tfmToPostJoint.linear().col(2);

	const unsigned int nColumns = _enabledParameters.sum();
	Eigen::MatrixXd jacobian(6, nColumns);
	int columnNo = 0;
	// tau
	if (_enabledParameters(0)) {
		jacobian.block<3, 1>(0, columnNo) = -sin(qi) * jointAxis.cross(posToEnd);
		jacobian.block<3, 1>(3, columnNo) = -sin(qi) * jointAxis;
		columnNo++;
	}
	// sigma
	if (_enabledParameters(1)) {
		jacobian.block<3, 1>(0, columnNo) = -cos(qi) * jointAxis.cross(posToEnd);
		jacobian.block<3, 1>(3, columnNo) = -cos(qi) * jointAxis;
		columnNo++;
	}

	return jacobian;
}

void EncoderParameterCalibration::doStep(const Eigen::VectorXd& step) {
	unsigned int enabledParameterNo = 0;
	Eigen::Vector2d parameterVector = Eigen::Vector2d::Zero();
	for (int parameterNo = 0; parameterNo < _enabledParameters.rows(); parameterNo++)
		if (_enabledParameters(parameterNo)) {
			parameterVector(parameterNo) = step(enabledParameterNo);
			enabledParameterNo++;
		}

	_correction = _correction + parameterVector;
}

}
}
