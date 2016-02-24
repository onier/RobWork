/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_MATH_POLYNOMIALSOLVER_HPP_
#define RW_MATH_POLYNOMIALSOLVER_HPP_

/**
 * @file PolynomialSolver.hpp
 *
 * \copydoc rwsim::contacts::PolynomialSolver
 */

#include "Polynomial.hpp"

#include <vector>
#include <complex>

namespace rw {
namespace math {
//! @addtogroup rw_math

//! @{
/**
 * @brief Solution of real and complex polynomial equations with Laguerre's Method.
 */
class PolynomialSolver {
public:
	/**
	 * @brief Create solver for the real polynomial equation given by a list of coefficients.
	 * @param coefficients [in] a list of real coefficients ordered from highest-order term to lowest-order term.
	 */
	PolynomialSolver(const std::vector<double> &coefficients);

	/**
	 * @brief Create solver for the complex polynomial equation given by a list of complex coefficients.
	 * @param coefficients [in] a list of complex coefficients ordered from highest-order term to lowest-order term.
	 */
	PolynomialSolver(const std::vector<std::complex<double> > &coefficients);

	/**
	 * @brief Destructor
	 */
	virtual ~PolynomialSolver();

	/**
	 * @brief Use a specific initial guess for a root.
	 * @param guess [in] a complex initial guess for the algorithm.
	 */
	void setInitialGuess(std::complex<double> guess = 0);

	/**
	 * @brief Get all real solutions of the equation.
	 * @return a list of real solutions.
	 */
	std::vector<double> getRealSolutions();

	/**
	 * @brief Get all solutions of the equation including complex solutions.
	 * @return a list of complex solutions.
	 */
	std::vector<std::complex<double> > getSolutions();

	std::complex<double> evaluate(std::complex<double> x, double &err) const;
	std::complex<double> evaluateDerivative(std::complex<double> x, std::size_t times = 1);

	PolynomialSolver* deflate(std::complex<double> x) const;
	std::complex<double> laguerre(std::complex<double> x);

	PolynomialSolver* getDerivative(std::size_t times = 1);

private:
	std::vector<std::complex<double> > _coefficients;
	PolynomialSolver* _derivative;
	std::complex<double> _guess;
	bool _isComplex;
};
//! @}
} /* namespace math */
} /* namespace rw */
#endif /* RW_MATH_POLYNOMIALSOLVER_HPP_ */
