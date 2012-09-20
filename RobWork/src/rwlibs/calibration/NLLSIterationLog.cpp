/*
 * NLLSIterationLog.cpp
 *
 *  Created on: Sep 20, 2012
 *      Author: bing
 */

#include "NLLSIterationLog.hpp"

namespace rwlibs {
namespace calibration {

NLLSIterationLog::NLLSIterationLog(int iterationNumber, bool isSingular, double conditionNumber, double residualNorm, double stepNorm) :
		_iterationNumber(iterationNumber), _isSingular(isSingular), _conditionNumber(conditionNumber), _residualNorm(residualNorm), _stepNorm(stepNorm) {

}

int NLLSIterationLog::getIterationNumber() const {
	return _iterationNumber;
}

bool NLLSIterationLog::isSingular() const {
	return _isSingular;
}

double NLLSIterationLog::getConditionNumber() const {
	return _conditionNumber;
}

double NLLSIterationLog::getResidualNorm() const {
	return _residualNorm;
}

double NLLSIterationLog::getStepNorm() const {
	return _stepNorm;
}

}
}
