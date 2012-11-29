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

	virtual double getParameterValue(int parameterIndex) const = 0;

	virtual void setParameterValue(int parameterIndex, double value) = 0;

	virtual int getParameterCount() const = 0;

	virtual int getEnabledParameterCount() const = 0;
	
	virtual bool isParameterEnabled(int parameterIndex) const = 0;
	
	virtual void setParameterEnabled(int parameterIndex, bool isEnabled) = 0;

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
