/*
 * IterativeSolver.hpp
 *
 *  Created on: Sep 12, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_ITERATIVESOLVER_HPP_
#define RWLIBS_CALIBRATION_ITERATIVESOLVER_HPP_

#include <Eigen/Core>

namespace rwlibs {
namespace calibration {

class IterativeSolver {
public:
	IterativeSolver();

	virtual ~IterativeSolver();

	void solve();

	virtual void computeJacobian(Eigen::MatrixXd& jacobian) = 0;

	virtual void computeResiduals(Eigen::VectorXd& residuals) = 0;

	virtual void takeStep(const Eigen::VectorXd& step) = 0;

protected:
	int _maxIterations;
	double _threshold;
};

}
}

#endif /* RWLIBS_CALIBRATION_ITERATIVESOLVER_HPP_ */
