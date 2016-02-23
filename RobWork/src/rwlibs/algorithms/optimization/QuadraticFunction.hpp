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

#ifndef RWLIBS_ALGORITHMS_OPTIMIZATION_QUADRATICFUNCTION_HPP_
#define RWLIBS_ALGORITHMS_OPTIMIZATION_QUADRATICFUNCTION_HPP_

/**
 * @file QuadraticFunction.hpp
 *
 * \copydoc rwlibs::optimization::QuadraticFunction
 */

#include <rw/math/Function.hpp>

#include <Eigen/Eigen>

namespace rwlibs {
namespace algorithms {
//! @addtogroup algorithms

//! @{
/**
 * @brief Representation of a quadratic function \f${\bf f}({\bf x})={\bf x}^T {\bf A}{\bf x} + {\bf b}^T {\bf x} + c\f$.
 */
template<class T = double>
class QuadraticFunction: public rw::math::Function1Diff<T, Eigen::Matrix<T, Eigen::Dynamic, 1>, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > {
public:
	//! @brief The vector type
	typedef Eigen::Matrix<T, Eigen::Dynamic, 1> Vector;

	//! @brief The Matrix type
	typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Matrix;

	/**
	 * @brief Construct new quadratic function.
	 * @param A [in] the quadratic coefficient matrix.
	 * @param b [in] (optional) the linear vector of coefficients.
	 * @param c [in] (optional) the constant term.
	 */
	QuadraticFunction(const Matrix& A, const Vector& b = Vector::Zero(), double c = 0):
		_A(A), _b(b), _c(c)
	{
	}

	//! @brief Destructor.
	virtual ~QuadraticFunction() {}

	//! @copydoc rw::math::Function::f
	virtual double f(Vector q) {
		return q.dot(_A*q)+_b.dot(q)+_c;
	}

	//! @copydoc rw::math::Function1Diff::df
	virtual Vector df(Vector q) {
		return 2*_A*q+_b;
	}

	/**
	 * @brief Get the quadratic coefficient matrix.
	 * @return the matrix.
	 */
	const Matrix& A() const {
		return _A;
	}

	/**
	 * @brief Get the linear term.
	 * @return the linear term.
	 */
	const Matrix& b() const {
		return _b;
	}

	/**
	 * @brief Get the constant term.
	 * @return the constant term.
	 */
	double c() const {
		return _c;
	}

private:
	const Matrix _A;
	const Vector _b;
	const double _c;
};
//! @}
} /* namespace algorithms */
} /* namespace rwlibs */
#endif /* RWLIBS_ALGORITHMS_OPTIMIZATION_QUADRATICFUNCTION_HPP_ */
