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

#ifndef RW_MODELS_FIXEDWHEEL_HPP_
#define RW_MODELS_FIXEDWHEEL_HPP_

#include "Wheel.hpp"
#include "RevoluteJoint.hpp"

namespace rw { namespace models {

class FixedWheel: public RevoluteJoint, public Wheel {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<FixedWheel> Ptr;

	/**
	 * @brief Constructs a fixed standard wheel
	 *
	 * @param name [in] Name of the wheel
	 * @param transform [in] Static transform of the wheel
	 * @param radius [in] Radius of the wheel
	 */
	FixedWheel(const std::string& name, const math::Transform3D<>& transform, double radius);

	//! @brief destructor
	~FixedWheel();

	/**
	 * @brief Infinite joint bounds are always returned for a wheel
	 * @return the lower and upper bound of this joint
	 */
	std::pair<math::Q, math::Q> getBounds() const;

	/**
	 * @brief Set the radius of the wheel
	 *
	 * @param radius [in] Radius of the wheel
	 */
	 void setRadius(double radius);

	/**
	 * @brief Return radius of the wheel
	 *
	 * @return The radius of the wheel
	 */
	 double getRadius() const;

	 //! @copydoc Wheel::isStearable
	 bool isStearable() const;

private:
	 double _radius;
};

}} // end namespaces

#endif // end include guard
