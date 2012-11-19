/*
 * NLLSSolver.hpp
 *
 *  Created on: Sep 12, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_NLLSSOLVER_HPP_
#define RWLIBS_CALIBRATION_NLLSSOLVER_HPP_

#include "NLLSIterationLog.hpp"
#include "NLLSSolverLog.hpp"
#include "NLLSSystem.hpp"
#include <Eigen/Core>
#include <Eigen/SVD>
#include <QtCore>

namespace rwlibs {
namespace calibration {

class NLLSSolver {
public:
	typedef rw::common::Ptr<NLLSSolver> Ptr;

	NLLSSolver(NLLSSystem::Ptr system);

	virtual ~NLLSSolver();

	NLLSSystem::Ptr getSystem() const;

	NLLSSolverLog::Ptr getLog() const;

	NLLSIterationLog iterate();

	void solve();

	void solve(double acceptThreshold, int maxIterations);

	Eigen::MatrixXd estimateCovariance() const;

protected:
	NLLSSystem::Ptr _system;
	NLLSSolverLog::Ptr _log;
	Eigen::MatrixXd _jacobian;
	Eigen::JacobiSVD<Eigen::MatrixXd> _jacobianSvd;
	Eigen::VectorXd _residuals;
	Eigen::VectorXd _step;
};

}
}

#endif /* RWLIBS_CALIBRATION_NLLSSOLVER_HPP_ */
