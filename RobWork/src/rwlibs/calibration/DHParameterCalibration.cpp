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
		_isEnabled(true), _isApplied(false), _joint(joint), _correction(
				rw::models::DHParameterSet::get(_joint.get())->isParallel() ?
						rw::models::DHParameterSet(0.0, 0.0, 0.0, 0.0, true) :
						rw::models::DHParameterSet(0.0, 0.0, 0.0, 0.0, rw::models::DHParameterSet::get(_joint.get())->getType())) {

}

DHParameterCalibration::DHParameterCalibration(rw::models::Joint::Ptr joint, const rw::models::DHParameterSet& correction) :
		_isEnabled(true), _isApplied(false), _joint(joint), _correction(correction) {

}

DHParameterCalibration::~DHParameterCalibration() {

}

bool DHParameterCalibration::isEnabled() const {
	return _isEnabled;
}

void DHParameterCalibration::apply() {
	if (!_isEnabled)
		RW_THROW("Not enabled.");
	if (_isApplied)
		RW_THROW("Already applied.");

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

	_isApplied = true;
}

void DHParameterCalibration::revert() {
	if (!_isEnabled)
		RW_THROW("Not enabled.");
	if (!_isApplied)
		RW_THROW("Not applied.");

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

	_isApplied = false;
}

void DHParameterCalibration::correct(rw::kinematics::State& state) {

}

bool DHParameterCalibration::isApplied() const {
	return _isApplied;
}

rw::models::Joint::Ptr DHParameterCalibration::getJoint() const {
	return _joint;
}

rw::models::DHParameterSet DHParameterCalibration::getCorrection() const {
	return _correction;
}

void DHParameterCalibration::correct(const rw::models::DHParameterSet& correction) {
	bool wasApplied = _isApplied;
	if (_isEnabled && _isApplied)
		revert();

	double a = _correction.a() + correction.a(), alpha = _correction.alpha() + correction.alpha();
	if (_correction.isParallel()) {
		double b = _correction.b() + correction.b(), beta = _correction.beta() + correction.beta();
		_correction = rw::models::DHParameterSet(alpha, a, beta, b, true);
	} else {
		double d = _correction.d() + correction.d(), theta = _correction.theta() + correction.theta();
		_correction = rw::models::DHParameterSet(alpha, a, d, theta, _correction.getType());
	}

	if (_isEnabled && wasApplied)
		apply();
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

}
}
