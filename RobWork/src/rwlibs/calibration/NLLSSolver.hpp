/*
 * NLLSSolver.hpp
 *
 *  Created on: Sep 12, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_NLLSSOLVER_HPP_
#define RWLIBS_CALIBRATION_NLLSSOLVER_HPP_

#include "NLLSIterationLog.hpp"
#include "NLLSSystem.hpp"
#include <Eigen/Core>
#include <Eigen/SVD>

namespace rwlibs {
namespace calibration {

class NLLSSolver {
public:
	typedef rw::common::Ptr<NLLSSolver> Ptr;

	NLLSSolver(NLLSSystem::Ptr system);

	virtual ~NLLSSolver();

	NLLSSystem::Ptr getSystem() const;

	const Eigen::MatrixXd& getJacobian() const;

	const Eigen::JacobiSVD<Eigen::MatrixXd>& getJacobianSvd() const;

	const Eigen::VectorXd& getResiduals() const;

	const Eigen::VectorXd& getStep() const;

	const std::vector<NLLSIterationLog>& getIterationLogs() const;

	NLLSIterationLog iterate();

	const std::vector<NLLSIterationLog>& solve();

	const std::vector<NLLSIterationLog>& solve(double acceptThreshold, int maxIterations);

protected:
	NLLSSystem::Ptr _system;
	Eigen::MatrixXd _jacobian;
	Eigen::JacobiSVD<Eigen::MatrixXd> _jacobianSvd;
	Eigen::VectorXd _residuals;
	Eigen::VectorXd _step;
	std::vector<NLLSIterationLog> _iterationLogs;
};

}
}

#endif /* RWLIBS_CALIBRATION_NLLSSOLVER_HPP_ */
