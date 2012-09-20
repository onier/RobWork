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
		_system(system) {

}

NLLSSolver::~NLLSSolver() {

}

NLLSSystem::Ptr NLLSSolver::getSystem() const {
	return _system;
}

const Eigen::MatrixXd& NLLSSolver::getJacobian() const {
	return _jacobian;
}

const Eigen::JacobiSVD<Eigen::MatrixXd>& NLLSSolver::getJacobianSvd() const {
	return _jacobianSvd;
}

const Eigen::VectorXd& NLLSSolver::getResiduals() const {
	return _residuals;
}

const Eigen::VectorXd& NLLSSolver::getStep() const {
	return _step;
}

const std::vector<NLLSIterationLog>& NLLSSolver::getIterationLogs() const {
	return _iterationLogs;
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
	int iterationNumber = _iterationLogs.size() + 1;
	Eigen::VectorXd singularValues = _jacobianSvd.singularValues();
	bool isSingular = (singularValues.rows() != _jacobianSvd.nonzeroSingularValues());
	double conditionNumber = singularValues(0) / singularValues(singularValues.rows() - 1);
	double residualNorm = _residuals.norm();
	double stepNorm = _step.norm();
	NLLSIterationLog iterationLog(iterationNumber, isSingular, conditionNumber, residualNorm, stepNorm);
	_iterationLogs.push_back(iterationLog);
	std::cout << "Iteration " << iterationNumber << " completed. Singular: " << (isSingular ? "Yes" : "No") << ". Condition: " << conditionNumber
			<< ". ||Residuals||: " << residualNorm << ". ||Step||: " << stepNorm << "." << std::endl;
//	std::cout << "Jacobian (" << _jacobian.rows() << "x" << _jacobian.cols() << "):" << std::endl;
//	std::cout << _jacobian.block(0, 0, _jacobian.rows() > 5 ? 6 : _jacobian.rows(), _jacobian.cols()) << std::endl;
	//			std::cout << "Residuals:\t" << residuals.segment(0, residuals.rows() > 6 ? 12 : 6).transpose() << std::endl;
	//			std::cout << "Step: \t" << step.transpose() << std::endl;
	//			std::cout << "---" << std::endl;
//	std::cin.ignore();

	return iterationLog;
}

const std::vector<NLLSIterationLog>& NLLSSolver::solve() {
	return solve(1e-14, 100);
}

const std::vector<NLLSIterationLog>& NLLSSolver::solve(double acceptThreshold, int maxIterationCount) {
	while (true) {
		NLLSIterationLog iterationLog = iterate();

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

	return _iterationLogs;
}

}
}
