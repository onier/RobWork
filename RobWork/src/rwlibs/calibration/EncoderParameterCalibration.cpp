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

EncoderParameterCalibration::EncoderParameterCalibration(rw::models::JointDevice::Ptr jointDevice, rw::models::Joint::Ptr joint) :
		_jointDevice(jointDevice), _joint(joint), _parameters(Eigen::VectorXd::Zero(getCorrectionFunctionCount())), _enabledParameters(Eigen::VectorXi::Ones(getCorrectionFunctionCount())) {
	// Find joint number.
	const std::vector<rw::models::Joint*> joints = jointDevice->getJoints();
	_jointNo = std::find(joints.begin(), joints.end(), joint.get()) - joints.begin();
}

EncoderParameterCalibration::EncoderParameterCalibration(rw::models::JointDevice::Ptr jointDevice, rw::models::Joint::Ptr joint,
		const Eigen::VectorXd& parameters) :
		_jointDevice(jointDevice), _joint(joint), _parameters(parameters), _enabledParameters(Eigen::VectorXi::Ones(getCorrectionFunctionCount())) {
	// Find joint number.
	const std::vector<rw::models::Joint*> joints = jointDevice->getJoints();
	_jointNo = std::find(joints.begin(), joints.end(), joint.get()) - joints.begin();
}

EncoderParameterCalibration::~EncoderParameterCalibration() {

}

rw::models::Joint::Ptr EncoderParameterCalibration::getJoint() const {
	return _joint;
}

void EncoderParameterCalibration::setEnabledParameters(bool tau, bool sigma) {
	_enabledParameters << tau, sigma;
}

QDomElement EncoderParameterCalibration::toXml(QDomDocument& document) {
	QDomElement element = document.createElement("EncoderParameterCalibration");

	element.setAttribute("joint", QString::fromStdString(_joint->getName()));

	element.setAttribute("tau", QString("%1").arg(_parameters(0), 0, 'g', 16));
	element.setAttribute("sigma", QString("%1").arg(_parameters(1), 0, 'g', 16));

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
	q[_jointNo] = q[_jointNo] + _parameters.cwiseProduct(computeCorrectionFunctionVector(q[_jointNo])).sum();
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

	const int nEnabledParameters = _enabledParameters.sum();
	Eigen::MatrixXd jacobian(6, nEnabledParameters);
	Eigen::VectorXd functionVector = computeCorrectionFunctionVector(qi);
	for (int parameterNo = 0; parameterNo < nEnabledParameters; parameterNo++) {
		jacobian.block<3, 1>(0, parameterNo) = functionVector(parameterNo) * jointAxis.cross(posToEnd);
		jacobian.block<3, 1>(3, parameterNo) = functionVector(parameterNo) * jointAxis;
	}

	return jacobian;
}

void EncoderParameterCalibration::doStep(const Eigen::VectorXd& step) {
	const int nParameters = _enabledParameters.rows();
	unsigned int enabledParameterNo = 0;
	Eigen::VectorXd parametersStep = Eigen::VectorXd::Zero(nParameters);
	for (int parameterNo = 0; parameterNo < nParameters; parameterNo++)
		if (_enabledParameters(parameterNo)) {
			parametersStep(parameterNo) = step(enabledParameterNo);
			enabledParameterNo++;
		}

	_parameters = _parameters + parametersStep;
}

int EncoderParameterCalibration::getCorrectionFunctionCount() const {
	return 2;
}

Eigen::VectorXd EncoderParameterCalibration::computeCorrectionFunctionVector(const double& q) {
	const unsigned int nEnabledParameters = _enabledParameters.sum();
	Eigen::VectorXd corrections(nEnabledParameters);
	int parameterNo = 0;
	// tau
	if (_enabledParameters(0)) {
		corrections(parameterNo) = -sin(q);
		parameterNo++;
	}
	// sigma
	if (_enabledParameters(1)) {
		corrections(parameterNo) = -cos(q);
		parameterNo++;
	}
	return corrections;
}

}
}
