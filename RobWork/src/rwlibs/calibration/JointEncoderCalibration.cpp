/*
* JointEncoderCalibration.cpp
*
*  Created on: Dec 20, 2012
*      Author: bing
*/

#include "JointEncoderCalibration.hpp"

namespace rwlibs {
	namespace calibration {
		int JointEncoderCalibration::PARAMETER_TAU = 0;
		int JointEncoderCalibration::PARAMETER_SIGMA = 1;

		class JointEncoderMapping : public rw::math::Function1Diff<> {
		public:
			JointEncoderMapping(const CalibrationParameterSet& parameterSet) : _parameterSet(parameterSet) {
			}

			virtual double x(double q) {
				double corrected = q;
				if (_parameterSet(JointEncoderCalibration::PARAMETER_TAU).isEnabled())
					corrected -= _parameterSet(JointEncoderCalibration::PARAMETER_TAU) * sin(q);
				if (_parameterSet(JointEncoderCalibration::PARAMETER_SIGMA).isEnabled())
					corrected -= _parameterSet(JointEncoderCalibration::PARAMETER_SIGMA) * cos(q);
				return corrected;
			}

			virtual double dx(double q) {
				return 0;
			}

		private:
			CalibrationParameterSet _parameterSet;
		};

		JointEncoderCalibration::JointEncoderCalibration(rw::models::JointDevice::Ptr device, rw::models::Joint::Ptr joint) : CalibrationBase(CalibrationParameterSet(2)), _device(device), _joint(joint) {
			
		}

		JointEncoderCalibration::~JointEncoderCalibration() {

		}

		rw::models::JointDevice::Ptr JointEncoderCalibration::getDevice() const {
			return _device;
		}

		rw::models::Joint::Ptr JointEncoderCalibration::getJoint() const {
			return _joint;
		}

		void JointEncoderCalibration::doApply() {
			CalibrationParameterSet parameterSet = getParameterSet();
			rw::math::Function1Diff<>::Ptr jointMapping = rw::common::ownedPtr(new JointEncoderMapping(parameterSet)).cast<rw::math::Function1Diff<> >();
			_joint->setJointMapping(jointMapping);
		}

		void JointEncoderCalibration::doRevert() {
			_joint->removeJointMapping();
		}
	}
}
