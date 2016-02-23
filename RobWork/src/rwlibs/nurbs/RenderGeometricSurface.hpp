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

#ifndef RWLIBS_NURBS_RENDERGEOMETRICSURFACE_HPP_
#define RWLIBS_NURBS_RENDERGEOMETRICSURFACE_HPP_

/**
 * @file RenderGeometricSurface.hpp
 *
 * \copydoc rwlibs::nurbs::RenderGeometricSurface
 */

#include "GeometricSurface.hpp"
#include <rw/graphics/Render.hpp>
#include <rwlibs/opengl/RenderGeometry.hpp>

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class RenderGeometricSurface: public rw::graphics::Render {
public:
    //! @brief smart pointer type to this class
	typedef rw::common::Ptr<RenderGeometricSurface> Ptr;

	/**
	 * @brief Constructs a RenderGeometricSurface object
	 *
	 * Constructs a RenderGeometricSurface object to visualize a Parametric surface in 3D.
	 *
	 * @param surface [in] the GeometricSurface to draw
	 */
	RenderGeometricSurface(GeometricSurface* surface);

    /**
     * @brief Destructor
     */
    virtual ~RenderGeometricSurface();

    /**
     * @brief Sets color of the object
     * @param r [in] red color component
     * @param g [in] green color component
     * @param b [in] blue color component
     */
    void setColor(float r, float g, float b);

    //! @copydoc rw::graphics::Render::draw
    void draw(const rw::graphics::DrawableNode::RenderInfo& info,
    		rw::graphics::DrawableNode::DrawType type,
    		double alpha) const;

    rw::math::Vector3D<float> getColor() const;

private:
    rwlibs::opengl::RenderGeometry* _geomRender;
    GeometricSurface* _surface;
    float _r, _g, _b;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_RENDERGEOMETRICSURFACE_HPP_ */
