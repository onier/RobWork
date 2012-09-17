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

NLLSSolver::NLLSSolver(NLLSProblem::Ptr problem) :
		_problem(problem), _maxIterations(100), _threshold(1e-14), _iterationNo(0) {

}

NLLSSolver::~NLLSSolver() {

}

void NLLSSolver::iterate() {
	_iterationNo++;

	_problem->computeJacobian(_jacobian);
	_problem->computeResiduals(_residuals);

	Eigen::JacobiSVD<Eigen::MatrixXd> jacobianSvd = _jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

	_step = jacobianSvd.solve(-_residuals);
	_steps.push_back(_step);

	// Compute singularity-test, norm of step and residuals, rate of convergence and condition number.
	bool isSingular = (jacobianSvd.singularValues().rows() != jacobianSvd.nonzeroSingularValues());
	double residualsNorm = _residuals.norm();
	double stepNorm = _step.norm();
	double stepErrorNorm = (_jacobian * _step - _residuals).norm();
	double roc = std::numeric_limits<double>::signaling_NaN();
	if (_iterationNo > 2) {
		roc = log(_steps[_iterationNo - 1].norm() / _steps[_iterationNo - 2].norm()) / log(_steps[_iterationNo - 2].norm() / _steps[_iterationNo - 3].norm());
	}
	Eigen::VectorXd singularValues = jacobianSvd.singularValues();
	double conditionNumber = singularValues(0) / singularValues(singularValues.rows() - 1);

	//			std::cout << "---" << std::endl;
	std::cout << "Iteration " << _iterationNo << " completed. Singular: " << (isSingular ? "Yes" : "No") << ". Condition: " << conditionNumber
			<< ". ||Residuals||: " << residualsNorm << ". ||Step||: " << stepNorm << ". ||StepError||: " << stepErrorNorm << ". RoC: " << roc << "."
			<< std::endl;
//	std::cout << "Jacobian (" << _jacobian.rows() << "x" << _jacobian.cols() << "):" << std::endl;
//	std::cout << _jacobian.block(0, 0, _jacobian.rows() > 5 ? 6 : _jacobian.rows(), _jacobian.cols()) << std::endl;
	//			std::cout << "Residuals:\t" << residuals.segment(0, residuals.rows() > 6 ? 12 : 6).transpose() << std::endl;
	//			std::cout << "Step: \t" << step.transpose() << std::endl;
//	std::cin.ignore();

	if (isSingular)
		RW_THROW("Singular Jacobian.");
	if (isnan(stepNorm))
		RW_THROW("NaN step.");
	if (isinf(stepNorm))
		RW_THROW("Infinite step.");

	_problem->takeStep(_step);
}

void NLLSSolver::solve() {
	while (true) {
		iterate();

		if (_step.norm() <= _threshold)
			break;

		if (_maxIterations > 0 && _iterationNo > _maxIterations)
			RW_THROW("Iteration limit exceeded.");
	}
}

}
}
