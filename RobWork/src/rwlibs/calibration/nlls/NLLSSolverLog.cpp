/*
 * NLLSSolverLog.cpp
 *
 *  Created on: Sep 21, 2012
 *      Author: bing
 */

#include "NLLSSolverLog.hpp"

namespace rwlibs {
namespace calibration {

NLLSSolverLog::NLLSSolverLog() {

}

NLLSIterationLog NLLSSolverLog::addIteration(const Eigen::MatrixXd& jacobian, const Eigen::JacobiSVD<Eigen::MatrixXd>& jacobianSvd, const Eigen::VectorXd& residuals,
		const Eigen::VectorXd& step) {
	_jacobian = jacobian;
	_jacobianSvd = jacobianSvd;
	_residuals = residuals;
	_step = step;

	int iterationNumber = _iterationLogs.size() + 1;
	Eigen::VectorXd singularValues = _jacobianSvd.singularValues();
	bool isSingular = (singularValues.rows() != _jacobianSvd.nonzeroSingularValues());
	double conditionNumber = singularValues(0) / singularValues(singularValues.rows() - 1);
	double residualNorm = _residuals.norm();
	double stepNorm = _step.norm();

	NLLSIterationLog iterationLog(iterationNumber, isSingular, conditionNumber, residualNorm, stepNorm);
	_iterationLogs.push_back(iterationLog);
	return iterationLog;
}

const Eigen::MatrixXd& NLLSSolverLog::getJacobian() const {
	return _jacobian;
}

const Eigen::JacobiSVD<Eigen::MatrixXd>& NLLSSolverLog::getJacobianSvd() const {
	return _jacobianSvd;
}

const Eigen::VectorXd& NLLSSolverLog::getResiduals() const {
	return _residuals;
}

const Eigen::VectorXd& NLLSSolverLog::getStep() const {
	return _step;
}

const std::vector<NLLSIterationLog>& NLLSSolverLog::getIterationLogs() const {
	return _iterationLogs;
}

}
}
