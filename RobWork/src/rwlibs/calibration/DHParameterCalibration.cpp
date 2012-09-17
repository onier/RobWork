/*
 * DHParameterCalibration.cpp
 *
 *  Created on: Aug 28, 2012
 *      Author: bing
 */

#include "DHParameterCalibration.hpp"

namespace rwlibs {
namespace calibration {

DHParameterCalibration::DHParameterCalibration(rw::models::Joint::Ptr joint) :
		_joint(joint), _correction(
				rw::models::DHParameterSet::get(_joint.get())->isParallel() ?
						rw::models::DHParameterSet(0.0, 0.0, 0.0, 0.0, true) :
						rw::models::DHParameterSet(0.0, 0.0, 0.0, 0.0, rw::models::DHParameterSet::get(_joint.get())->getType())), _enabledParameters(
				Eigen::Vector4i::Ones()) {

}

DHParameterCalibration::DHParameterCalibration(rw::models::Joint::Ptr joint, const rw::models::DHParameterSet& correction) :
		_joint(joint), _correction(correction), _enabledParameters(Eigen::Vector4i::Ones()) {

}

DHParameterCalibration::~DHParameterCalibration() {

}

rw::models::Joint::Ptr DHParameterCalibration::getJoint() const {
	return _joint;
}

rw::models::DHParameterSet DHParameterCalibration::getCorrection() const {
	return _correction;
}

void DHParameterCalibration::setEnabledParameters(bool a, bool length, bool alpha, bool angle) {
	_enabledParameters << a, length, alpha, angle;
}

QDomElement DHParameterCalibration::toXml(QDomDocument& document) {
	QDomElement element = document.createElement("DHParameterCalibration");

	element.setAttribute("joint", QString::fromStdString(_joint->getName()));

	element.setAttribute("type", QString::fromStdString(_correction.getType()));
	element.setAttribute("alpha", QString("%1").arg(_correction.alpha(), 0, 'g', 16));
	element.setAttribute("a", QString("%1").arg(_correction.a(), 0, 'g', 16));
	if (_correction.isParallel()) {
		element.setAttribute("offset", QString("%1").arg(_correction.beta(), 0, 'g', 16));
		element.setAttribute("b", QString("%1").arg(_correction.b(), 0, 'g', 16));
	} else {
		element.setAttribute("offset", QString("%1").arg(_correction.theta(), 0, 'g', 16));
		element.setAttribute("d", QString("%1").arg(_correction.d(), 0, 'g', 16));
	}

	return element;
}

DHParameterCalibration::Ptr DHParameterCalibration::fromXml(const QDomElement& element, rw::kinematics::StateStructure::Ptr stateStructure) {
	if (!element.hasAttribute("joint"))
		RW_THROW("\"joint\" attribute missing.");
	QString jointName = element.attribute("joint");

	rw::models::Joint::Ptr joint = (rw::models::Joint*) stateStructure->findFrame(jointName.toStdString());
	if (joint.isNull())
		RW_THROW(QString("Joint \"%1\" not found.").arg(jointName).toStdString());

	if (!element.hasAttribute("alpha"))
		RW_THROW(QString("Joint \"%1\" needs \"alpha\" attribute.").arg(jointName).toStdString());
	double alpha = element.attribute("alpha").toDouble();

	if (!element.hasAttribute("a"))
		RW_THROW(QString("Joint \"%1\" needs \"a\" attribute.").arg(jointName).toStdString());
	double a = element.attribute("a").toDouble();

	double d;
	bool isParallel;
	if (!(element.hasAttribute("b") || element.hasAttribute("d")))
		RW_THROW(QString("Joint \"%1\" needs \"b\" or \"d\" attribute.").arg(jointName).toStdString());
	if (element.hasAttribute("b")) {
		d = element.attribute("b").toDouble();
		isParallel = true;
	} else {
		d = element.attribute("d").toDouble();
		isParallel = false;
	}

	if (!element.hasAttribute("offset"))
		RW_THROW(QString("Joint \"%1\" needs \"offset\" attribute.").arg(jointName).toStdString());
	double offset = element.attribute("offset").toDouble();

	return rw::common::ownedPtr(
			new DHParameterCalibration(joint,
					isParallel ? rw::models::DHParameterSet(alpha, a, offset, d, isParallel) : rw::models::DHParameterSet(alpha, a, d, offset)));

}

void DHParameterCalibration::doApply() {
	rw::models::DHParameterSet current = *rw::models::DHParameterSet::get(_joint.get());

	double alpha = current.alpha() + _correction.alpha();
	double a = current.a() + _correction.a();
	if (current.isParallel()) {
		double beta = current.beta() + _correction.beta();
		double b = current.b() + _correction.b();
		rw::models::DHParameterSet dhParameterSet(alpha, a, beta, b, true);
		rw::models::DHParameterSet::set(dhParameterSet, _joint.get());
		_joint->setFixedTransform(rw::math::Transform3D<double>::DHHGP(alpha, a, beta, b));
	} else {
		double d = current.d() + _correction.d();
		double theta = current.theta() + _correction.theta();
		rw::models::DHParameterSet dhParameterSet(alpha, a, d, theta, current.getType());
		rw::models::DHParameterSet::set(dhParameterSet, _joint.get());
		_joint->setFixedTransform(rw::math::Transform3D<double>::DH(alpha, a, d, theta));
	}
}

void DHParameterCalibration::doRevert() {
	rw::models::DHParameterSet current = *rw::models::DHParameterSet::get(_joint.get());
	double alpha = current.alpha() - _correction.alpha();
	double a = current.a() - _correction.a();
	if (current.isParallel()) {
		double beta = current.beta() - _correction.beta();
		double b = current.b() - _correction.b();
		rw::models::DHParameterSet dhParameterSet(alpha, a, beta, b, true);
		rw::models::DHParameterSet::set(dhParameterSet, _joint.get());
		_joint->setFixedTransform(rw::math::Transform3D<double>::DHHGP(alpha, a, beta, b));
	} else {
		double d = current.d() - _correction.d();
		double theta = current.theta() - _correction.theta();
		rw::models::DHParameterSet dhParameterSet(alpha, a, d, theta, current.getType());
		rw::models::DHParameterSet::set(dhParameterSet, _joint.get());
		_joint->setFixedTransform(rw::math::Transform3D<double>::DH(alpha, a, d, theta));
	}
}

void DHParameterCalibration::doCorrect(rw::kinematics::State& state) {

}

int DHParameterCalibration::doGetParameterCount() const {
	return _enabledParameters.sum();
}

Eigen::MatrixXd DHParameterCalibration::doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
		const rw::kinematics::State& state) {
	const Eigen::Affine3d tfmToPreLink = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), _joint->getParent(state), state);
	const Eigen::Affine3d tfmLink = _joint->getFixedTransform();
	const Eigen::Affine3d tfmToPostLink = tfmToPreLink * tfmLink;
	const Eigen::Affine3d tfmJoint = _joint->getJointTransform(state);
	const Eigen::Affine3d tfmPostJoint = rw::kinematics::Kinematics::frameTframe(_joint.get(), measurementFrame.get(), state);
	const Eigen::Affine3d tfmToEnd = tfmToPostLink * tfmJoint * tfmPostJoint;
	const bool isParallel = rw::models::DHParameterSet::get(_joint.get())->isParallel();

	const unsigned int nColumns = _enabledParameters.sum();
	Eigen::MatrixXd jacobian(6, nColumns);
	int columnNo = 0;
	// a
	if (_enabledParameters(0)) {
		jacobian.block<3, 1>(0, columnNo) = tfmToPostLink.linear().col(0);
		jacobian.block<3, 1>(3, columnNo) = Eigen::Vector3d::Zero();
		columnNo++;
	}
	// b/d
	if (_enabledParameters(1)) {
		jacobian.block<3, 1>(0, columnNo) = tfmToPreLink.linear().col(isParallel ? 1 : 2);
		jacobian.block<3, 1>(3, columnNo) = Eigen::Vector3d::Zero();
		columnNo++;
	}
	// alpha
	if (_enabledParameters(2)) {
		Eigen::Vector3d xAxisToPost = tfmToPostLink.linear().col(0);
		Eigen::Vector3d tlPostToEnd = tfmToEnd.translation() - tfmToPostLink.translation();
		jacobian.block<3, 1>(0, columnNo) = xAxisToPost.cross(tlPostToEnd);
		jacobian.block<3, 1>(3, columnNo) = xAxisToPost;
		columnNo++;
	}
	// beta/theta
	if (_enabledParameters(3)) {
		Eigen::Vector3d yzAxisToPre = tfmToPreLink.linear().col(isParallel ? 1 : 2);
		Eigen::Vector3d tlPreToEnd = tfmToEnd.translation() - tfmToPreLink.translation();
		jacobian.block<3, 1>(0, columnNo) = yzAxisToPre.cross(tlPreToEnd);
		jacobian.block<3, 1>(3, columnNo) = yzAxisToPre;
		columnNo++;
	}

	return jacobian;
}

void DHParameterCalibration::doStep(const Eigen::VectorXd& step) {
	unsigned int enabledParameterNo = 0;
	Eigen::Vector4d parameterVector = Eigen::Vector4d::Zero();
	for (int parameterNo = 0; parameterNo < _enabledParameters.rows(); parameterNo++)
		if (_enabledParameters(parameterNo)) {
			parameterVector(parameterNo) = step(enabledParameterNo);
			enabledParameterNo++;
		}

	bool wasApplied = isApplied();
	if (wasApplied)
		revert();

	double a = _correction.a() + parameterVector(0), alpha = _correction.alpha() + parameterVector(2);
	if (_correction.isParallel()) {
		double b = _correction.b() + parameterVector(1), beta = _correction.beta() + parameterVector(3);
		_correction = rw::models::DHParameterSet(alpha, a, beta, b, true);
	} else {
		double d = _correction.d() + parameterVector(1), theta = _correction.theta() + parameterVector(3);
		_correction = rw::models::DHParameterSet(alpha, a, d, theta, _correction.getType());
	}

	if (wasApplied)
		apply();
}

}
}
