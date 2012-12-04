/*
* DHParameterCalibration.cpp
*
*  Created on: Aug 28, 2012
*      Author: bing
*/

#include "DHParameterCalibration.hpp"

namespace rwlibs {
	namespace calibration {
		int DHParameterCalibration::PARAMETER_A = 0;
		int DHParameterCalibration::PARAMETER_B = 1;
		int DHParameterCalibration::PARAMETER_D = 2;
		int DHParameterCalibration::PARAMETER_ALPHA = 3;
		int DHParameterCalibration::PARAMETER_BETA = 4;
		int DHParameterCalibration::PARAMETER_THETA = 5;

		DHParameterCalibration::DHParameterCalibration(rw::models::Joint::Ptr joint) : CalibrationBase(CalibrationParameterSet(6)), _joint(joint), _originalSet(*rw::models::DHParameterSet::get(joint.get())), _isParallel(_originalSet.isParallel()) {
			CalibrationParameterSet parameterSet = getParameterSet();
			if (_isParallel) {
				parameterSet(PARAMETER_D).setEnabled(false);
				parameterSet(PARAMETER_THETA).setEnabled(false);
				setParameterSet(parameterSet);
			} else {
				parameterSet(PARAMETER_B).setEnabled(false);
				parameterSet(PARAMETER_BETA).setEnabled(false);
				setParameterSet(parameterSet);
			
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
			CalibrationParameterSet parameterSet = getParameterSet();
			_originalSet = *rw::models::DHParameterSet::get(_joint.get());
			const double a = parameterSet(PARAMETER_A).isEnabled() ? (_originalSet.a() + parameterSet(PARAMETER_A)) : _originalSet.a();
			const double alpha = parameterSet(PARAMETER_ALPHA).isEnabled() ? (_originalSet.alpha() + parameterSet(PARAMETER_ALPHA)) : _originalSet.alpha();
			if (_originalSet.isParallel()) {
				const double b = parameterSet(PARAMETER_B).isEnabled() ? (_originalSet.b() + parameterSet(PARAMETER_B)) : _originalSet.b();
				const double beta = parameterSet(PARAMETER_BETA).isEnabled() ? (_originalSet.beta() + parameterSet(PARAMETER_BETA)) : _originalSet.beta();
				const rw::models::DHParameterSet correctedSet(alpha, a, beta, b, true);
				rw::models::DHParameterSet::set(correctedSet, _joint.get());
				_joint->setFixedTransform(rw::math::Transform3D<double>::DHHGP(alpha, a, beta, b));
			} else {
				const double d = parameterSet(PARAMETER_D).isEnabled() ? (_originalSet.d() + parameterSet(PARAMETER_D)) : _originalSet.d();
				const double theta = parameterSet(PARAMETER_THETA).isEnabled() ? (_originalSet.theta() + parameterSet(PARAMETER_THETA)) : _originalSet.theta();
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
