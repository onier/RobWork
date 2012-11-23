/*
 * JacobianBase.hpp
 *
 *  Created on: Nov 22, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_JACOBIANBASE_HPP_
#define RWLIBS_CALIBRATION_JACOBIANBASE_HPP_

#include "Jacobian.hpp"

namespace rwlibs {
namespace calibration {
/** @addtogroup calibration */
/*@{*/

class JacobianBase: public Jacobian {
public:
	typedef rw::common::Ptr<JacobianBase> Ptr;

	/**
	 * @brief Destructor.
	 */
	virtual ~JacobianBase();
	
	/**
	 * @copydoc Jacobian::isEnabled()
	 */
	virtual bool isEnabled() const;

	/**
	 * @copydoc Jacobian::setEnabled()
	 */
	virtual void setEnabled(bool isEnabled);
	
	/**
	 * @copydoc Jacobian::getParameterCount()
	 */
	virtual int getParameterCount() const;

	virtual bool isParameterEnabled(int parameterIndex);

	virtual void setParameterEnabled(int parameterIndex, bool isEnabled);
	
	/**
	 * @copydoc Jacobian::computeJacobian()
	 */
	virtual Eigen::MatrixXd computeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame, const rw::kinematics::State& state);
	
	virtual void takeStep(const Eigen::VectorXd& step);

protected:
	/**
	 * @brief Constructor.
	 */
	JacobianBase(int parameterCount);

	/**
	 * @brief Subclass implementation of computeJacobian().
	 */
	virtual Eigen::MatrixXd doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
			const rw::kinematics::State& state) = 0;

	/**
	 * @brief Subclass implementation of takeStep().
	 */
	virtual void doTakeStep(const Eigen::VectorXd& step) = 0;

private:
	bool _isEnabled;
	Eigen::VectorXi _enabledParameters;
};

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_JACOBIANBASE_HPP_ */