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
		_jointDevice(jointDevice), _joint(joint), _parameters(Eigen::VectorXd::Zero(getCorrectionFunctionCount())), _lockedParameters(Eigen::VectorXi::Zero(getCorrectionFunctionCount())) {
	// Find joint number.
	const std::vector<rw::models::Joint*> joints = jointDevice->getJoints();
	_jointIndex = std::find(joints.begin(), joints.end(), joint.get()) - joints.begin();
}

EncoderParameterCalibration::EncoderParameterCalibration(rw::models::JointDevice::Ptr jointDevice, rw::models::Joint::Ptr joint,
		const Eigen::VectorXd& parameters) :
		_jointDevice(jointDevice), _joint(joint), _parameters(parameters), _lockedParameters(Eigen::VectorXi::Zero(getCorrectionFunctionCount())) {
	// Find joint number.
	const std::vector<rw::models::Joint*> joints = jointDevice->getJoints();
	_jointIndex = std::find(joints.begin(), joints.end(), joint.get()) - joints.begin();
}

EncoderParameterCalibration::~EncoderParameterCalibration() {

}

rw::models::Joint::Ptr EncoderParameterCalibration::getJoint() const {
	return _joint;
}

void EncoderParameterCalibration::setLockedParameters(bool tau, bool sigma) {
	_lockedParameters << tau, sigma;
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
	q[_jointIndex] = q[_jointIndex] + _parameters.cwiseProduct(computeCorrectionFunctionVector(q[_jointIndex])).sum();
	_jointDevice->setQ(q, state);
}

int EncoderParameterCalibration::doGetParameterCount() const {
	return _lockedParameters.rows() - _lockedParameters.sum();
}

Eigen::MatrixXd EncoderParameterCalibration::doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
		const rw::kinematics::State& state) {
	// Get joint value.
	const rw::math::Q q = _jointDevice->getQ(state);
	const double qi = q[_jointIndex];

	// Prepare transformations.
	const Eigen::Affine3d tfmToPostJoint = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), _joint.get(), state);
	const Eigen::Affine3d tfmPostJoint = rw::kinematics::Kinematics::frameTframe(_joint.get(), measurementFrame.get(), state);
	const Eigen::Affine3d tfmToEnd = tfmToPostJoint * tfmPostJoint;
	const Eigen::Vector3d posToEnd = tfmToEnd.translation() - tfmToPostJoint.translation();
	const Eigen::Vector3d jointAxis = tfmToPostJoint.linear().col(2);

	const int nUnlockedParameters = getParameterCount();
	Eigen::MatrixXd jacobian(6, nUnlockedParameters);
	Eigen::VectorXd functionVector = computeCorrectionFunctionVector(qi);
	for (int parameterNo = 0; parameterNo < nUnlockedParameters; parameterNo++) {
		jacobian.block<3, 1>(0, parameterNo) = functionVector(parameterNo) * jointAxis.cross(posToEnd);
		jacobian.block<3, 1>(3, parameterNo) = functionVector(parameterNo) * jointAxis;
	}

	return jacobian;
}

void EncoderParameterCalibration::doStep(const Eigen::VectorXd& step) {
	const int parameterCount = _lockedParameters.rows();
	unsigned int unlockedParameterIndex = 0;
	Eigen::VectorXd parametersStep = Eigen::VectorXd::Zero(parameterCount);
	for (int parameterIndex = 0; parameterIndex < parameterCount; parameterIndex++)
		if (!_lockedParameters(parameterIndex))
			parametersStep(parameterIndex) = step(unlockedParameterIndex++);

	_parameters = _parameters + parametersStep;
}

int EncoderParameterCalibration::getCorrectionFunctionCount() const {
	return 2;
}

Eigen::VectorXd EncoderParameterCalibration::computeCorrectionFunctionVector(const double& q) {
	const unsigned int parameterCount = getParameterCount();
	Eigen::VectorXd corrections(parameterCount);
	int parameterIndex = 0;
	// tau
	if (!_lockedParameters(0))
		corrections(parameterIndex++) = -sin(q);
	// sigma
	if (!_lockedParameters(1))
		corrections(parameterIndex) = -cos(q);
	return corrections;
}

}
}
