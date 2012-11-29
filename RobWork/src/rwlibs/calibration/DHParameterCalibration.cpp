/*
* DHParameterCalibration.cpp
*
*  Created on: Aug 28, 2012
*      Author: bing
*/

#include "DHParameterCalibration.hpp"

namespace rwlibs {
	namespace calibration {

		DHParameterCalibration::DHParameterCalibration(rw::models::Joint::Ptr joint) : CalibrationBase(6), _joint(joint), _originalSet(*rw::models::DHParameterSet::get(joint.get())), _isParallel(_originalSet.isParallel()) {
			if (_isParallel) {
				setParameterEnabled(PARAMETER_D, false), setParameterEnabled(PARAMETER_THETA, false);
			} else {
				setParameterEnabled(PARAMETER_B, false), setParameterEnabled(PARAMETER_BETA, false);
			
				// Warn if b/beta representation is not used for close to parallel joints.
				const double alphaParallelThreshold = 10 * rw::math::Deg2Rad;
				if (abs(_originalSet.alpha()) < alphaParallelThreshold)
					RW_WARN("DH alpha parameter close to zero for joint \"" << joint->getName() << "\". Singularities might occur, consider using b/beta parameters instead of d/theta.");
			}
		}

		DHParameterCalibration::~DHParameterCalibration() {

		}

		rw::models::Joint::Ptr DHParameterCalibration::getJoint() const {
			return _joint;
		}

		void DHParameterCalibration::doApply() {
			_originalSet = *rw::models::DHParameterSet::get(_joint.get());
			const double a = isParameterEnabled(PARAMETER_A) ? (_originalSet.a() + getParameterValue(PARAMETER_A)) : _originalSet.a();
			const double alpha = isParameterEnabled(PARAMETER_ALPHA) ? (_originalSet.alpha() + getParameterValue(PARAMETER_ALPHA)) : _originalSet.alpha();
			if (_originalSet.isParallel()) {
				const double b = isParameterEnabled(PARAMETER_B) ? (_originalSet.b() + getParameterValue(PARAMETER_B)) : _originalSet.b();
				const double beta = isParameterEnabled(PARAMETER_BETA) ? (_originalSet.beta() + getParameterValue(PARAMETER_BETA)) : _originalSet.beta();
				const rw::models::DHParameterSet correctedSet(alpha, a, beta, b, true);
				rw::models::DHParameterSet::set(correctedSet, _joint.get());
				_joint->setFixedTransform(rw::math::Transform3D<double>::DHHGP(alpha, a, beta, b));
			} else {
				const double d = isParameterEnabled(PARAMETER_D) ? (_originalSet.d() + getParameterValue(PARAMETER_D)) : _originalSet.d();
				const double theta = isParameterEnabled(PARAMETER_THETA) ? (_originalSet.theta() + getParameterValue(PARAMETER_THETA)) : _originalSet.theta();
				const rw::models::DHParameterSet correctedSet(alpha, a, d, theta, _originalSet.getType());
				rw::models::DHParameterSet::set(correctedSet, _joint.get());
				_joint->setFixedTransform(rw::math::Transform3D<double>::DH(alpha, a, d, theta));
			}
		}

		void DHParameterCalibration::doRevert() {
			if (_originalSet.isParallel()) {
				rw::models::DHParameterSet::set(_originalSet, _joint.get());
				_joint->setFixedTransform(rw::math::Transform3D<double>::DHHGP(_originalSet.alpha(), _originalSet.a(), _originalSet.beta(), _originalSet.b()));
			} else {
				rw::models::DHParameterSet::set(_originalSet, _joint.get());
				_joint->setFixedTransform(rw::math::Transform3D<double>::DH(_originalSet.alpha(), _originalSet.a(), _originalSet.d(), _originalSet.theta()));
			}
		}

	}
}
