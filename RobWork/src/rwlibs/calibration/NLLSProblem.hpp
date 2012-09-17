/*
 * NLLSProblem.hpp
 *
 *  Created on: Sep 17, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_JACOBIAN_HPP_
#define RWLIBS_CALIBRATION_JACOBIAN_HPP_

#include <Eigen/Core>
#include <rw/common.hpp>

namespace rwlibs {
namespace calibration {

class NLLSProblem {
public:
	typedef rw::common::Ptr<NLLSProblem> Ptr;

	virtual ~NLLSProblem();

	virtual void computeJacobian(Eigen::MatrixXd& jacobian) = 0;

	virtual void computeResiduals(Eigen::VectorXd& residuals) = 0;

	virtual void takeStep(const Eigen::VectorXd& step) = 0;
};

}
}


#endif /* RWLIBS_CALIBRATION_JACOBIAN_HPP_ */
