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
 * Calibrations has the concept of \i locking. Locked calibrations cannot change state
 */
class Calibration {
public:
	typedef rw::common::Ptr<Calibration> Ptr;

	/**
	 * @brief Destructor.
	 */
	virtual ~Calibration();

	/**
	 * @brief Test if calibration is locked.
	 * @return True if locked, false otherwise
	 */
	virtual bool isLocked() const;

	/**
	 * @brief Lock or unlock calibration.
	 * @param[in] isLocked True to lock, false to unlock.
	 */
	virtual void setLocked(bool isLocked);

	/**
	 * @brief Test if calibration is applied.
	 * @return True if applied, false otherwise
	 */
	virtual bool isApplied() const;

	/**
	 * @brief Apply calibration.
	 *
	 * Exception is thrown if calibration is locked or already applied.
	 */
	void apply();

	/**
	 * @brief Revert calibration.
	 *
	 * Exception is thrown if calibration is locked or not applied.
	 */
	void revert();

	void correct(rw::kinematics::State& state);

	/**
	 * @brief Returns the number of parameters describing the calibration.
	 * @return Number of parameters
	 */
	int getParameterCount() const;

	/**
	 * @brief Compute the Jacobian matrix.
	 *
	 * Exception is thrown if calibration is locked, not applied or getParameterCount() returns 0.
	 *
	 * @see	getParameterCount()
	 * @param[in]	referenceFrame	Reference frame from which partial derivatives are seen.
	 * @param[in]	targetFrame		Target frame of which the partial derivatives are described.
	 * @param[in]	state			State of the work cell.
	 * @return Jacobian matrix
	 */
	Eigen::MatrixXd computeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame, const rw::kinematics::State& state);

	void takeStep(const Eigen::VectorXd& step);

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

	virtual void doCorrect(rw::kinematics::State& state) = 0;

	/**
	 * @brief Subclass implementation of getParameterCount().
	 */
	virtual int doGetParameterCount() const = 0;

	/**
	 * @brief Subclass implementation of computeJacobian().
	 */
	virtual Eigen::MatrixXd doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
			const rw::kinematics::State& state) = 0;

	virtual void doTakeStep(const Eigen::VectorXd& step) = 0;

private:
	bool _isLocked;
	bool _isApplied;
};

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_CALIBRATION_HPP_ */
