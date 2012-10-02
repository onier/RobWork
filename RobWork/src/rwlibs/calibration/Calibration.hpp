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
 */
class Calibration {
public:
	typedef rw::common::Ptr<Calibration> Ptr;

	/**
	 * @brief Destructor.
	 */
	virtual ~Calibration();

	virtual bool isLocked() const;

	virtual void setLocked(bool isLocked);

	/**
	 * @brief Test if calibration is applied.
	 * @return True if applied, false otherwise.
	 */
	virtual bool isApplied() const;

	/**
	 * @brief Apply calibration.
	 */
	void apply();

	/**
	 * @brief Revert calibration.
	 */
	void revert();

	void correct(rw::kinematics::State& state);

	int getParameterCount() const;

	Eigen::MatrixXd computeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& state);

	void step(const Eigen::VectorXd& step);

protected:
	/**
	 * @brief Protected constructor.
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

	virtual int doGetParameterCount() const = 0;

	virtual Eigen::MatrixXd doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
			const rw::kinematics::State& state) = 0;

	virtual void doStep(const Eigen::VectorXd& step) = 0;

private:
	bool _isLocked;
	bool _isApplied;
};

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_CALIBRATION_HPP_ */
