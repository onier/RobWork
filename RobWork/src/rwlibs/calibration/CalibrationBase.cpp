/*
* CalibrationBase.cpp
*
*  Created on: Nov 26, 2012
*      Author: bing
*/

#include "CalibrationBase.hpp"

namespace rwlibs {
	namespace calibration {

		CalibrationBase::~CalibrationBase() {

		}
		
		bool CalibrationBase::isEnabled() const {
			return _isEnabled;
		}

		void CalibrationBase::setEnabled(bool isEnabled) {
			RW_ASSERT(!isApplied());

			_isEnabled = isEnabled;
		}

		double CalibrationBase::getParameterValue(int parameterIndex) const {
			RW_ASSERT(parameterIndex < getParameterCount());
			RW_ASSERT(isParameterEnabled(parameterIndex));

			return isParameterEnabled(parameterIndex) * _parameters(parameterIndex);
		}

		void CalibrationBase::setParameterValue(int parameterIndex, double parameterValue) {
			RW_ASSERT(parameterIndex < getParameterCount());
			RW_ASSERT(isParameterEnabled(parameterIndex));
			RW_ASSERT(!isApplied());

			_parameters(parameterIndex) = parameterValue;
		}

		int CalibrationBase::getParameterCount() const {
			return _enabledParameters.rows();
		}

		int CalibrationBase::getEnabledParameterCount() const {
			return isEnabled() ? _enabledParameters.sum() : 0;
		}

		bool CalibrationBase::isParameterEnabled(int parameterIndex) const {
			RW_ASSERT(parameterIndex < getParameterCount());

			return _enabledParameters(parameterIndex) != 0;
		}

		void CalibrationBase::setParameterEnabled(int parameterIndex, bool isEnabled) {
			RW_ASSERT(parameterIndex < getParameterCount());
			RW_ASSERT(!isApplied());

			_enabledParameters(parameterIndex) = isEnabled;
		}

		bool CalibrationBase::isApplied() const {
			return _isApplied;
		}

		void CalibrationBase::apply() {
			RW_ASSERT(isEnabled());
			RW_ASSERT(!isApplied());

			doApply();

			_isApplied = true;

			RW_ASSERT(isApplied());
		}

		void CalibrationBase::revert() {
			RW_ASSERT(isEnabled());
			RW_ASSERT(isApplied());

			doRevert();

			_isApplied = false;

			RW_ASSERT(!isApplied());
		}

		CalibrationBase::CalibrationBase(int parameterCount) :
			_parameters(Eigen::VectorXd::Zero(parameterCount)), _isEnabled(true), _enabledParameters(Eigen::VectorXi::Ones(parameterCount)), _isApplied(false) {

		}

	}
}
