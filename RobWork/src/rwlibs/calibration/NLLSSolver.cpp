/*
 * NLLSSolver.cpp
 *
 *  Created on: Sep 12, 2012
 *      Author: bing
 */

#include "NLLSSolver.hpp"

#include <Eigen/SVD>
#include <rw/common.hpp>

namespace rwlibs {
namespace calibration {

NLLSSolver::NLLSSolver(NLLSSystem::Ptr system) :
		_system(system), _log(rw::common::ownedPtr(new NLLSSolverLog())) {

}

NLLSSolver::~NLLSSolver() {

}

NLLSSystem::Ptr NLLSSolver::getSystem() const {
	return _system;
}

NLLSSolverLog::Ptr NLLSSolver::getLog() const {
	return _log;
}

NLLSIterationLog NLLSSolver::iterate() {
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

	return _log->addIteration(_jacobian, _jacobianSvd, _residuals, _step);
}

void NLLSSolver::solve() {
	return solve(1e-14, 100);
}

void NLLSSolver::solve(double acceptThreshold, int maxIterationCount) {
	while (true) {
		NLLSIterationLog iterationLog = iterate();

		std::cout << "Iteration " << iterationLog.getIterationNumber() << " completed. Singular: " << (iterationLog.isSingular() ? "Yes" : "No")
				<< ". Condition: " << iterationLog.getConditionNumber() << ". ||Residuals||: " << iterationLog.getResidualNorm() << ". ||Step||: "
				<< iterationLog.getStepNorm() << "." << std::endl;

		// Verify iteration.
		if (iterationLog.isSingular())
			RW_THROW("Singular Jacobian.");
		if (isnan(iterationLog.getStepNorm()))
			RW_THROW("NaN step.");
		if (isinf(iterationLog.getStepNorm()))
			RW_THROW("Infinite step.");

		// Stop iterating if step is below accepted threshold.
		if (iterationLog.getStepNorm() <= acceptThreshold)
			break;

		// Throw exception if iteration limit has been reached.
		if (maxIterationCount > 0 && iterationLog.getIterationNumber() >= maxIterationCount)
			RW_THROW("Iteration limit reached.");
	}
}

}
}
