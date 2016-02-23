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

#ifndef RWLIBS_ALGORITHMS_OPTIMIZATION_QUADRATICOPTIMIZATIONPROBLEM_HPP_
#define RWLIBS_ALGORITHMS_OPTIMIZATION_QUADRATICOPTIMIZATIONPROBLEM_HPP_

/**
 * @file QuadraticOptimizationProblem.hpp
 *
 * \copydoc rwlibs::algorithms::QuadraticOptimizationProblem
 */

#include "QuadraticFunction.hpp"

namespace rwlibs {
namespace algorithms {
//! @addtogroup algorithms

//! @{
/**
 * @brief Optimization of .
 *
 * The problem is of the form:
 * \f$\min _{x} {\bf q}({\bf x})=\frac{1}{2} {\bf x}^T {\bf Q}{\bf x} + {\bf c}^T {\bf x}\f$
 * where \f${\bf A} {\bf x} \leq {\bf b}\f$.
 *
 * In general the QP problem is known to be NP-hard. If \f${\bf Q}\f$ is Positive Definite a solution can be found in polynomial time \f$O(n^k)$\f.
 */
class QuadraticOptimizationProblem {
public:
	QuadraticOptimizationProblem(const QuadraticFunction& fct);
	virtual ~QuadraticOptimizationProblem();
};
//! @}
} /* namespace algorithms */
} /* namespace rwlibs */
#endif /* RWLIBS_ALGORITHMS_OPTIMIZATION_QUADRATICOPTIMIZATIONPROBLEM_HPP_ */
