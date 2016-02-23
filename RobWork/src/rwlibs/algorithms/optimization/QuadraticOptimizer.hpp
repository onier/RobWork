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

#ifndef RWLIBS_ALGORITHMS_OPTIMIZATION_QUADRATICOPTIMIZER_HPP_
#define RWLIBS_ALGORITHMS_OPTIMIZATION_QUADRATICOPTIMIZER_HPP_

/**
 * @file QuadraticOptimizer.hpp
 *
 * \copydoc rwlibs::algorithms::QuadraticOptimizer
 */

#include "Optimizer.hpp"

namespace rwlibs {
namespace algorithms {
//! @addtogroup INSERT_DOC_GROUP

//! @{
/**
 * @brief INSERT_SHORT_DESCRIPTION
 */
template<class T = double>
class QuadraticOptimizer: public Optimizer<QuadraticFunction<T> > {
public:
	QuadraticOptimizer();
	virtual ~QuadraticOptimizer();

	virtual void minimize(const QuadraticFunction<T>& f) = 0;
};
//! @}
} /* namespace algorithms */
} /* namespace rwlibs */
#endif /* RWLIBS_ALGORITHMS_OPTIMIZATION_QUADRATICOPTIMIZER_HPP_ */
