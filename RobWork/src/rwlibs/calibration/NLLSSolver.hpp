/*
 * NLLSSolver.hpp
 *
 *  Created on: Sep 12, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_NLLSSOLVER_HPP_
#define RWLIBS_CALIBRATION_NLLSSOLVER_HPP_

#include "NLLSProblem.hpp"
#include <vector>
#include <Eigen/Core>

namespace rwlibs {
namespace calibration {

class NLLSSolver {
public:
	NLLSSolver(NLLSProblem::Ptr problem);

	virtual ~NLLSSolver();

	void iterate();

	void solve();

protected:
	NLLSProblem::Ptr _problem;
	int _maxIterations;
	double _threshold;
	int _iterationNo;
	Eigen::MatrixXd _jacobian;
	Eigen::VectorXd _residuals;
	Eigen::VectorXd _step;
	std::vector<Eigen::VectorXd> _steps;
};

}
}

#endif /* RWLIBS_CALIBRATION_NLLSSOLVER_HPP_ */
