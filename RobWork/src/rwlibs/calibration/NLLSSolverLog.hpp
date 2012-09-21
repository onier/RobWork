/*
 * NLLSSolverLog.hpp
 *
 *  Created on: Sep 21, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_NLLSSOLVERLOG_HPP_
#define RWLIBS_CALIBRATION_NLLSSOLVERLOG_HPP_

#include "NLLSIterationLog.hpp"
#include <Eigen/Core>
#include <Eigen/SVD>
#include <rw/common.hpp>

namespace rwlibs {
namespace calibration {

class NLLSSolverLog {
public:
	typedef rw::common::Ptr<NLLSSolverLog> Ptr;

	NLLSSolverLog();

	NLLSIterationLog addIteration(const Eigen::MatrixXd& jacobian, const Eigen::JacobiSVD<Eigen::MatrixXd>& jacobianSvd, const Eigen::VectorXd& residuals,
			const Eigen::VectorXd& step);

	const Eigen::MatrixXd& getJacobian() const;

	const Eigen::JacobiSVD<Eigen::MatrixXd>& getJacobianSvd() const;

	const Eigen::VectorXd& getResiduals() const;

	const Eigen::VectorXd& getStep() const;

	const std::vector<NLLSIterationLog>& getIterationLogs() const;

private:
	Eigen::MatrixXd _jacobian;
	Eigen::JacobiSVD<Eigen::MatrixXd> _jacobianSvd;
	Eigen::VectorXd _residuals;
	Eigen::VectorXd _step;
	std::vector<NLLSIterationLog> _iterationLogs;
};

}
}

#endif /* RWLIBS_CALIBRATION_NLLSSOLVERLOG_HPP_ */
