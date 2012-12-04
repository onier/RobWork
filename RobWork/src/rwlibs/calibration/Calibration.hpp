/*
 * Calibration.hpp
 *
 *  Created on: Aug 30, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_CALIBRATION_HPP_
#define RWLIBS_CALIBRATION_CALIBRATION_HPP_

#include <Eigen/Core>
#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {
/** @addtogroup calibration */
/*@{*/

class CalibrationParameter {
public:
	CalibrationParameter() : _isEnabled(true), _value(0.0) {

	}

	CalibrationParameter(double initialValue) : _isEnabled(true), _value(initialValue) {

	}

	~CalibrationParameter() {

	}

	bool isEnabled() const {
		return _isEnabled;
	}

	void setEnabled(bool isEnabled) {
		_isEnabled = isEnabled;
	}
	
	operator double() const {
		return _value;
	}
	
	CalibrationParameter& operator =(const CalibrationParameter& other) {
		_isEnabled = other._isEnabled;
		_value = other._value;
		return *this;
	}
	
	CalibrationParameter& operator =(const double& value) {
		RW_ASSERT(isEnabled());
		_value = value;
		return *this;
	}
	
	double operator +(const double& value) {
		return _value + value;
	}
	
	CalibrationParameter operator +=(const double& value) {
		RW_ASSERT(isEnabled());
		this->operator=(this->operator+(value));
		return *this;
	}

private:
	bool _isEnabled;
	double _value;
};

class CalibrationParameterSet {
public:
	CalibrationParameterSet(int parameterCount) : _parameters(parameterCount) {

	}

	int getCount() const {
		return _parameters.size();
	}

	int getEnabledCount() const {
		const int count = getCount();
		int enabledCount = 0;
		for (int parameterIndex = 0; parameterIndex < count; parameterIndex++)
			if (_parameters[parameterIndex].isEnabled())
				enabledCount++;
		return enabledCount;
	}
	
	CalibrationParameter& operator ()(int parameterIndex) {
		RW_ASSERT(parameterIndex < getCount());
		return _parameters[parameterIndex];
	}
	
	const CalibrationParameter& operator ()(int parameterIndex) const {
		RW_ASSERT(parameterIndex < getCount());
		return _parameters[parameterIndex];
	}

private:
	std::vector<CalibrationParameter> _parameters;
};

/**
 * @brief Calibration represents a kinematic correction.
 *
 * Calibrations can be applied or reverted. When applied the kinematics of the workcell will be modified according to the calibration model.
 */
class Calibration {
public:
	typedef rw::common::Ptr<Calibration> Ptr;

	/**
	 * @brief Destructor.
	 */
	virtual ~Calibration();

	virtual bool isEnabled() const = 0;

	virtual void setEnabled(bool isEnabled) = 0;

	virtual CalibrationParameterSet getParameterSet() const = 0;

	virtual void setParameterSet(const CalibrationParameterSet& parameterSet) = 0;

	/**
	 * @brief Test if calibration is applied.
	 * @return True if applied, false otherwise
	 */
	virtual bool isApplied() const = 0;

	/**
	 * @brief Apply calibration.
	 *
	 * Exception is thrown if calibration is already applied.
	 */
	virtual void apply() = 0;

	/**
	 * @brief Revert calibration.
	 *
	 * Exception is thrown if calibration is not applied.
	 */
	virtual void revert() = 0;
};

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_CALIBRATION_HPP_ */
