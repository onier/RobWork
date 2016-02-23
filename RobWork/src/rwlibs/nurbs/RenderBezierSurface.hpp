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

#ifndef RWLIBS_NURBS_RENDERBEZIERSURFACE_HPP_
#define RWLIBS_NURBS_RENDERBEZIERSURFACE_HPP_

/**
 * @file RenderBezierSurface.hpp
 *
 * \copydoc rwlibs::nurbs::RenderBezierSurface
 */

#include <rw/graphics/Render.hpp>
#include "GeometricBezierSurface.hpp"
#include "RenderBezierCurve.hpp"

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class RenderBezierSurface: public rw::graphics::Render {
public:
    //! @brief smart pointer type to this class
	typedef rw::common::Ptr<RenderBezierSurface> Ptr;

	/**
	 * @brief Constructs a RenderBezierSurface object
	 *
	 * Constructs a RenderBezierSurface object to visualize a Bezier surface in 3 dimensions.
	 *
	 * @param curve [in] the BezierSurface to draw
	 */
	RenderBezierSurface(const GeometricBezierSurface &surface);

	/**
	 * @brief Destructor
	 */
	virtual ~RenderBezierSurface();

	//! @copydoc rw::graphics::Render::draw
	void draw(const rw::graphics::DrawableNode::RenderInfo& info,
			rw::graphics::DrawableNode::DrawType type,
			double alpha) const;

	void setColor(float r, float g, float b);

	rw::math::Vector3D<float> getColor() const;

private:
	GeometricBezierSurface _surface;
    float _r, _g, _b;
    std::vector<RenderBezierCurve*> _curvesU, _curvesV;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_RENDERBEZIERSURFACE_HPP_ */
