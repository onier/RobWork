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

#ifndef ROBWORK_SRC_RWLIBS_ALGORITHMS_OPTIMIZATION_OPTIMIZER_HPP_
#define ROBWORK_SRC_RWLIBS_ALGORITHMS_OPTIMIZATION_OPTIMIZER_HPP_

/**
 * @file Optimizer.hpp
 *
 * \copydoc rwlibs::algorithms::Optimizer
 */

namespace rwlibs {
namespace algorithms {
//! @addtogroup INSERT_DOC_GROUP

//! @{
/**
 * @brief INSERT_SHORT_DESCRIPTION
 */
template<class T = double, class F = rw::math::Function1Diff<RES_T,ARG_T,GRAD_T> >
class Optimizer {
public:
	Optimizer();
	virtual ~Optimizer();
	virtual ARG_T minimize(const rw::math::Function1Diff<RES_T,ARG_T,GRAD_T>& f) = 0;
};
//! @}
} /* namespace algorithms */
} /* namespace rwlibs */
#endif /* ROBWORK_SRC_RWLIBS_ALGORITHMS_OPTIMIZATION_OPTIMIZER_HPP_ */
