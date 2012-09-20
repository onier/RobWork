/*
 * NLLSIterationLog.hpp
 *
 *  Created on: Sep 20, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_NLLSITERATIONLOG_HPP_
#define RWLIBS_CALIBRATION_NLLSITERATIONLOG_HPP_

namespace rwlibs {
namespace calibration {

class NLLSIterationLog {
public:
	NLLSIterationLog(int iterationNumber, bool isSingular, double conditionNumber, double residualNorm, double stepNorm);

	int getIterationNumber() const;

	bool isSingular() const;

	double getConditionNumber() const;

	double getResidualNorm() const;

	double getStepNorm() const;

private:
	int _iterationNumber;
	bool _isSingular;
	double _conditionNumber;
	double _residualNorm;
	double _stepNorm;
};

}
}


#endif /* RWLIBS_CALIBRATION_NLLSITERATIONLOG_HPP_ */
