/*
 * CalibrationBase.hpp
 *
 *  Created on: Nov 26, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_CALIBRATIONBASE_HPP_
#define RWLIBS_CALIBRATION_CALIBRATIONBASE_HPP_

#include "Calibration.hpp"
#include <Eigen/Core>
#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {
/** @addtogroup calibration */
/*@{*/

/**
 * @copydoc Calibration
 */
class CalibrationBase: public Calibration {
public:
	typedef rw::common::Ptr<CalibrationBase> Ptr;

	/**
	 * @brief Destructor.
	 */
	virtual ~CalibrationBase();
	
	virtual bool isEnabled() const;

	virtual void setEnabled(bool isEnabled);

	virtual double getParameterValue(int parameterIndex) const;

	virtual void setParameterValue(int parameterIndex, double value);
	
	virtual int getParameterCount() const;
	
	virtual int getEnabledParameterCount() const;
	
	virtual bool isParameterEnabled(int parameterIndex) const;
	
	virtual void setParameterEnabled(int parameterIndex, bool isEnabled);

	/**
	 * @copydoc Calibration::isApplied()
	 */
	virtual bool isApplied() const;

	/**
	 * @copydoc Calibration::apply()
	 */
	virtual void apply();

	/**
	 * @copydoc Calibration::revert()
	 */
	virtual void revert();

protected:
	/**
	 * @brief Constructor.
	 */
	CalibrationBase(int parameterCount);

	/**
	 * @brief Subclass implementation of apply().
	 */
	virtual void doApply() = 0;

	/**
	 * @brief Subclass implementation of revert().
	 */
	virtual void doRevert() = 0;

private:
	Eigen::VectorXd _parameters;
	bool _isEnabled;
	Eigen::VectorXi _enabledParameters;
	bool _isApplied;
};

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_CALIBRATIONBASE_HPP_ */
