/*
 * NLLSNewtonSolver.cpp
 *
 *  Created on: Sep 12, 2012
 *      Author: bing
 */

#include "NLLSNewtonSolver.hpp"

#include <boost/math/special_functions/fpclassify.hpp>
#include <Eigen/SVD>
#include <rw/common.hpp>

namespace rwlibs {
namespace calibration {

const double stepConvergenceTolerance = 1e-14;
const int maxIterationCount = 100;

NLLSNewtonSolver::NLLSNewtonSolver(NLLSSystem::Ptr system) :
		_system(system) {

}

NLLSNewtonSolver::~NLLSNewtonSolver() {

}

NLLSSystem::Ptr NLLSNewtonSolver::getSystem() const {
	return _system;
}

const NLLSSolverLog& NLLSNewtonSolver::getLog() const {
	return _log;
}

NLLSIterationLog NLLSNewtonSolver::iterate() {
	// Compute Jacobian and residuals.
	_system->computeJacobian(_jacobian);
	_system->computeResiduals(_residuals);

	// Compute SVD of Jacobian.
	_jacobianSvd = _jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

	// Solve Jacobian * step = residuals.
	_step = _jacobianSvd.solve(-_residuals);

	// Apply step.
	_system->takeStep(_step);

	// Log iteration.
	NLLSIterationLog iterationLog = _log.addIteration(_jacobian, _jacobianSvd, _residuals, _step);

	// Verify iteration.
	if (iterationLog.isSingular())
		RW_THROW("Singular Jacobian.");
	if (boost::math::isnan(iterationLog.getStepNorm()))
		RW_THROW("NaN step.");
	if (boost::math::isinf(iterationLog.getStepNorm()))
		RW_THROW("Infinite step.");

	return iterationLog;
}

bool NLLSNewtonSolver::isConverged() const {
	return _step.norm() <= stepConvergenceTolerance;
}

void NLLSNewtonSolver::solve() {
	while (true) {
		NLLSIterationLog iterationLog = iterate();

		//std::cout << "Iteration " << iterationLog.getIterationNumber() << " completed. Singular: " << (iterationLog.isSingular() ? "Yes" : "No")
		//	<< ". Condition: " << iterationLog.getConditionNumber() << ". ||Residuals||: " << iterationLog.getResidualNorm() << ". ||Step||: "
		//	<< iterationLog.getStepNorm() << "." << std::endl;

		// Stop iterating if step is below accepted threshold.
		if (isConverged())
			break;

		// Throw exception if iteration limit has been reached.
		if (maxIterationCount > 0 && iterationLog.getIterationNumber() >= maxIterationCount)
			RW_THROW("Iteration limit reached.");
	}
}

Eigen::MatrixXd NLLSNewtonSolver::estimateCovarianceMatrix() const {
	// Eq. 15.4.20 from Numerical Recipes (covariance matrix of unknown variables)
	Eigen::MatrixXd V = _jacobianSvd.matrixV();
	Eigen::VectorXd singularValues = _jacobianSvd.singularValues();

	double eps = std::numeric_limits<double>::epsilon();
	double precision = eps * _jacobianSvd.rows() * singularValues(0);

	Eigen::MatrixXd covarianceMatrix(V.rows(), V.cols());
	for (int j = 0; j < V.rows(); j++) {
		for (int k = 0; k < V.rows(); k++) {
			double sum = 0;
			for (int i = 0; i < _jacobianSvd.cols(); i++) {
				if (singularValues(i) > precision)
					sum += (V(j, i) * V(k, i)) / (singularValues(i) * singularValues(i));
			}
			covarianceMatrix(j, k) = sum;
		}
	}

	return covarianceMatrix;
}

}
}
