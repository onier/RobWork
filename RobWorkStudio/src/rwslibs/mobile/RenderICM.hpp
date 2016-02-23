/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RENDERICM_HPP_
#define RENDERICM_HPP_

//! @file RenderICM.hpp

#include <list>
#include <vector>

#include <rw/math/Vector3D.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/graphics/Render.hpp>

namespace rws {
	//! @addtogroup rws
	//! @{
	/**
	 * @brief Render a circle for the instantaneous center of motion for mobile devices
	 */
	class RenderICM: public rw::graphics::Render
	{
	public:
		RenderICM(rw::math::Vector3D<> icm, double radius, rw::math::Vector3D<> normal, float angleResolution=10.0);

		/**
		 * @brief constructor for inactive render
		 */
		RenderICM();

		/**
		 * @brief destructor
		 */
		virtual ~RenderICM();

		/**
		 * @brief Updates location and radius
		 * @param icm Instantaneous center of motion
		 * @param radius The radius of the motion
		 * @param normal The normal for the circle
		 */
		void setICM(rw::math::Vector3D<> icm, double radius, rw::math::Vector3D<> normal);

		/**
		 * @brief Set inactive
		 */
		void setInactive();

		/**
		 * @brief set the color used for the model
		 * @param r [in] red color value
		 * @param g [in] green color value
		 * @param b [in] blue color value
		 */
		void setColor(double r, double g, double b);

		/**
		 * @brief clear the list of circles
		 */
		void clear();

        //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
        void draw(const rw::graphics::DrawableNode::RenderInfo& info,
                  rw::graphics::DrawableNode::DrawType type,
                  double alpha) const;

	private:
        bool _active;
		rw::math::Vector3D<> _icm;
		double _radius;
		rw::math::Vector3D<> _normal;
		float _stepSize;
		float _color[3];
	};

	//! @}
}

#endif /*RenderICM_HPP_*/
