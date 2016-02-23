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

#ifndef RW_MODELS_STEEREDWHEEL_HPP_
#define RW_MODELS_STEEREDWHEEL_HPP_

#include "Wheel.hpp"
#include "FixedWheel.hpp"
#include "RevoluteJoint.hpp"

namespace rw { namespace models {

class SteeredWheel: public RevoluteJoint, public Wheel {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<SteeredWheel> Ptr;

	/**
	 * @brief Constructs a centered orientational wheel.
	 * The wheel is steered around the z-axis, and drives along the x-axis.
	 * The wheel can be offset in the y-direction.
	 *
	 * @param name [in] Name of the wheel
	 * @param transform [in] Static transform of the Infinite joint bounds are always returned for a wheel
	 * @param wheel [in] The name of the fixed wheel
	 * @param offset [in] Optional offset of the wheel in the y-direction
	 */
	SteeredWheel(const std::string& name, const math::Transform3D<>& transform, const std::string &wheel, double offset = 0.0);

	//! @brief destructor
	~SteeredWheel();

    /**
     * @brief Sets joint bounds for the steering joint
     * @param bounds [in] the lower and upper bounds of the steering joint
     */
    void setBounds(const std::pair<const math::Q, const math::Q>& bounds);

	/**
	 * @brief Bounds for the steering mechanism
	 * @return the lower and upper bound of this joint
	 */
	std::pair<math::Q, math::Q> getBounds() const;

	//! @copydoc Joint::getJacobian
	void getJacobian(size_t row,
			size_t col,
			const math::Transform3D<>& joint,
			const math::Transform3D<>& tcp,
			const kinematics::State& state,
			math::Jacobian& jacobian) const;

	//! @copydoc Joint::getFixedTransform()
	rw::math::Transform3D<> getFixedTransform() const;

	//! @copydoc Joint::setFixedTransform()
	void setFixedTransform( const rw::math::Transform3D<>& t3d);

	//! @copydoc Joint::getJointTransform()
	math::Transform3D<> getJointTransform(const rw::kinematics::State& state) const;

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
	 double getRadius();

	 /**
	  * @brief Set the offset of the wheel
	  *
	  * @param radius [in] Offset of the wheel
	  */
	 void setOffset(double radius);

	 /**
	  * @brief Return offset of the wheel
	  *
	  * @return The offset of the wheel
	  */
	 double getOffset() const;

	 //! @copydoc Wheel::isStearable
	 bool isStearable() const;

	 void setWheel(FixedWheel* wheel);

	 FixedWheel* getWheel();

private:
	 void findWheel();

	 const std::string &_wheelName;
	 FixedWheel* _wheel;
	 double _radius;
	 double _offset;
	 rw::math::Transform3D<> _transform;
};

}} // end namespaces

#endif // end include guard
