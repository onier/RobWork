/*
 * IterativeSolver.cpp
 *
 *  Created on: Sep 12, 2012
 *      Author: bing
 */

#include "IterativeSolver.hpp"

#include <Eigen/SVD>
#include <rw/common.hpp>

namespace rwlibs {
namespace calibration {

IterativeSolver::IterativeSolver() :
		_maxIterations(100), _threshold(1e-14) {

}

IterativeSolver::~IterativeSolver() {

}

void IterativeSolver::solve() {
	int iterationNo = 0;
	std::vector<Eigen::VectorXd> steps;
	Eigen::MatrixXd jacobian;
	Eigen::VectorXd residuals;
	while (true) {
		iterationNo++;

		computeResiduals(residuals);
		computeJacobian(jacobian);

		Eigen::JacobiSVD<Eigen::MatrixXd> jacobianSvd = jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

		Eigen::VectorXd step = jacobianSvd.solve(-residuals);
		steps.push_back(step);

		// Compute singularity-test, norm of step and residuals, rate of convergence and condition number.
		bool isSingular = (jacobianSvd.singularValues().rows() != jacobianSvd.nonzeroSingularValues());
		double residualsNorm = residuals.norm();
		double stepNorm = step.norm();
		double stepErrorNorm = (jacobian * step - residuals).norm();
		double roc = std::numeric_limits<double>::signaling_NaN();
		if (iterationNo > 2) {
			roc = log(steps[iterationNo - 1].norm() / steps[iterationNo - 2].norm()) / log(steps[iterationNo - 2].norm() / steps[iterationNo - 3].norm());
		}
		Eigen::VectorXd singularValues = jacobianSvd.singularValues();
		double conditionNumber = singularValues(0) / singularValues(singularValues.rows() - 1);

//			std::cout << "---" << std::endl;
		std::cout << "Iteration " << iterationNo << " completed. Singular: " << (isSingular ? "Yes" : "No") << ". Condition: " << conditionNumber
				<< ". ||Residuals||: " << residualsNorm << ". ||Step||: " << stepNorm << ". ||StepError||: " << stepErrorNorm << ". RoC: " << roc << "."
				<< std::endl;
//			std::cout << "Jacobian (" << jacobian.rows() << "x" << jacobian.cols() << "):" << std::endl;
//			std::cout << jacobian.block(0, 0, jacobian.rows() > 5 ? 6 : jacobian.rows(), jacobian.cols()) << std::endl;
//			std::cout << "Residuals:\t" << residuals.segment(0, residuals.rows() > 6 ? 12 : 6).transpose() << std::endl;
//			std::cout << "Step: \t" << step.transpose() << std::endl;
//			std::cin.ignore();

		if (isSingular)
			RW_THROW("Singular Jacobian.");
		if (isnan(stepNorm))
			RW_THROW("NaN step.");
		if (isinf(stepNorm))
			RW_THROW("Infinite step.");

		takeStep(step);

		if (stepNorm <= _threshold) {
			computeJacobian(jacobian);
			computeResiduals(residuals);
			break;
		}

		if (_maxIterations > 0 && iterationNo > _maxIterations)
			RW_THROW("Iteration limit exceeded.");
	}
}

}
}
