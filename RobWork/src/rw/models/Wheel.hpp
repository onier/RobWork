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

#ifndef RW_MODELS_WHEEL_HPP_
#define RW_MODELS_WHEEL_HPP_

#include "Joint.hpp"

namespace rw { namespace models {

class Wheel {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<Wheel> Ptr;

	/**
	 * @brief Constructs Wheel
	 */
	Wheel() {}

	//! @brief destructor
	virtual ~Wheel() {}

	/**
	 * @brief Stearable wheels can be steered without changing the position of the wheel.
	 * @return true if wheel can be steared
	 */
	virtual bool isStearable() const = 0;
};

}} // end namespaces
#endif /* RW_MODELS_WHEEL_HPP_ */
