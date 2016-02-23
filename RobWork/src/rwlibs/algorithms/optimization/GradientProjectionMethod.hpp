/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RWLIBS_OPTIMIZATION_GRADIENTPROJECTIONMETHOD_HPP_
#define RWLIBS_OPTIMIZATION_GRADIENTPROJECTIONMETHOD_HPP_

/**
 * @file GradientProjectionMethod.hpp
 *
 * \copydoc rwlibs::optimization::GradientProjectionMethod
 */

#include "QuadraticOptimizer.hpp"

#include <Eigen/Eigen>

namespace rwlibs {
namespace algorithms {
//! @addtogroup algorithms

//! @{
/**
 * @brief Solver of Quadratic Programming problems using the Gradient Projection Method.
 *
 * The method is described in section 16.7 of Nocedal, J., Wright, S.J.: Numerical Optimization, 2nd edn. Springer, Heidelberg (2006)
 *
 * In this implementation it is assumed that the problem is convex and that the system matrix is positive semi-definite.
 * It might however work for indefinite matrices, but there is no guarantee that it will do so.
 */
class GradientProjectionMethod: public QuadraticOptimizer {
public:
	//! @brief Construct new GradientProjectionMethod.
	GradientProjectionMethod();

	//! @brief Destructor
	virtual ~GradientProjectionMethod();

	/**
	 * @brief Solve the Quadratic Programming problem
	 * \f$\min _{x} {\bf q}({\bf x})=\frac{1}{2} {\bf x}^T {\bf G}{\bf x} + {\bf x}^T {\bf c}\f$
	 * where \f${\bf l} \leq {\bf x} \leq {\bf u}\f$.
	 * @param f [in] the quadratic objective function.
	 * @param l [in] the lower limit for the result - use #negativeInfinity for specification of no limit.
	 * @param u [in] the upper limit for the result - use #positiveInfinity for specification of no limit.
	 * @param iterations [in] the maximum number of iterations.
	 * @param eps [in] the precision.
	 * @param x [in/out] the starting point for the search for the solution.
	 * @return the result \f${\bf x}\f$.
	 */
	void minimize(const QuadraticFunction& f, const Eigen::VectorXd& l, const Eigen::VectorXd& u, Eigen::VectorXd& x, std::size_t iterations, double eps) const;

	/**
	 * @brief Get the value used to represent negative infinity \f$-\infty\f$ for use in limit specification.
	 * @return a double representing a infinite negative value.
	 */
	static double negativeInfinity();

	/**
	 * @brief Get the value used to represent positive infinity \f$\infty\f$ for use in limit specification.
	 * @return a double representing a infinite positive value.
	 */
	static double positiveInfinity();

private:
	struct TimeCompare;
};
//! @}
} /* namespace algorithms */
} /* namespace rwlibs */
#endif /* RWLIBS_OPTIMIZATION_GRADIENTPROJECTIONMETHOD_HPP_ */
