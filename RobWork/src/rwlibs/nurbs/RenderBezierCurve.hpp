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

#ifndef RWLIBS_NURBS_RENDERBEZIERCURVE_HPP_
#define RWLIBS_NURBS_RENDERBEZIERCURVE_HPP_

/**
 * @file RenderBezierCurve.hpp
 *
 * \copydoc rwlibs::nurbs::RenderBezierCurve
 */

#include <rw/graphics/Render.hpp>
#include "GeometricBezierCurve.hpp"

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class RenderBezierCurve: public rw::graphics::Render {
public:
    //! @brief smart pointer type to this class
	typedef rw::common::Ptr<RenderBezierCurve> Ptr;

	/**
	 * @brief Constructs a RenderBezierCurve object
	 *
	 * Constructs a RenderBezierCurve object to visualize a Bezier curve in 3 dimensions.
	 *
	 * @param curve [in] the BezierCurve to draw
	 */
	RenderBezierCurve(const GeometricBezierCurve &curve);

	/**
	 * @brief Destructor
	 */
	virtual ~RenderBezierCurve();

	//! @copydoc rw::graphics::Render::draw
	void draw(const rw::graphics::DrawableNode::RenderInfo& info,
			rw::graphics::DrawableNode::DrawType type,
			double alpha) const;

	void setColor(float r, float g, float b);

	rw::math::Vector3D<float> getColor() const;

private:
	GeometricBezierCurve _curve;
    float _r, _g, _b;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_RENDERBEZIERCURVE_HPP_ */
