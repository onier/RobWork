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
 * Calibrations can be applied or reverted.
 */
class Calibration {
public:
	typedef rw::common::Ptr<Calibration> Ptr;

	/**
	 * @brief Destructor.
	 */
	virtual ~Calibration();

	/**
	 * @brief Test if calibration is applied.
	 * @return True if applied, false otherwise
	 */
	virtual bool isApplied() const;

	/**
	 * @brief Apply calibration.
	 *
	 * Exception is thrown if calibration is already applied.
	 */
	void apply();

	/**
	 * @brief Revert calibration.
	 *
	 * Exception is thrown if calibration is not applied.
	 */
	void revert();

	/**
	 * @brief Correct state according to calibration.
	 *
	 * Exception is thrown if calibration is not applied.
	 */
	void correctState(rw::kinematics::State& state);

protected:
	/**
	 * @brief Constructor.
	 */
	Calibration();

	/**
	 * @brief Subclass implementation of apply().
	 */
	virtual void doApply() = 0;

	/**
	 * @brief Subclass implementation of revert().
	 */
	virtual void doRevert() = 0;
	
	/**
	 * @brief Subclass implementation of correctState().
	 */
	virtual void doCorrectState(rw::kinematics::State& state) = 0;

private:
	bool _isApplied;
};

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_CALIBRATION_HPP_ */
