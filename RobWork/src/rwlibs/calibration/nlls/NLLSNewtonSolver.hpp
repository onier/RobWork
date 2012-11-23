/*
 * NLLSNewtonSolver.hpp
 *
 *  Created on: Sep 12, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_NLLSNEWTONSOLVER_HPP_
#define RWLIBS_CALIBRATION_NLLSNEWTONSOLVER_HPP_

#include "NLLSIterationLog.hpp"
#include "NLLSSolver.hpp"
#include "NLLSSystem.hpp"
#include <Eigen/Core>
#include <Eigen/SVD>
#include <QtCore>

namespace rwlibs {
namespace calibration {

class NLLSNewtonSolver: public NLLSSolver {
public:
	typedef rw::common::Ptr<NLLSNewtonSolver> Ptr;

	NLLSNewtonSolver(NLLSSystem::Ptr system);

	virtual ~NLLSNewtonSolver();

	virtual NLLSSystem::Ptr getSystem() const;

	virtual const NLLSSolverLog& getLog() const;

	virtual NLLSIterationLog iterate();

	virtual bool isConverged() const;

	virtual void solve();

	virtual Eigen::MatrixXd estimateCovarianceMatrix() const;

protected:
	NLLSSystem::Ptr _system;
	NLLSSolverLog _log;
	Eigen::MatrixXd _jacobian;
	Eigen::JacobiSVD<Eigen::MatrixXd> _jacobianSvd;
	Eigen::VectorXd _residuals;
	Eigen::VectorXd _step;
};

}
}

#endif /* RWLIBS_CALIBRATION_NLLSNEWTONSOLVER_HPP_ */
