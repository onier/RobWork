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

#ifndef RWLIBS_NURBS_RENDERPARAMETRICCURVE_HPP_
#define RWLIBS_NURBS_RENDERPARAMETRICCURVE_HPP_

/**
 * @file RenderParametricCurve.hpp
 *
 * \copydoc rwlibs::nurbs::RenderParametricCurve
 */

#include <rw/graphics/Render.hpp>
#include <rwlibs/nurbs/ParametricCurve.hpp>

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class RenderParametricCurve: public rw::graphics::Render {
public:
    //! @brief smart pointer type to this class
	typedef rw::common::Ptr<RenderParametricCurve> Ptr;

	/**
	 * @brief Constructs a RenderParametricCurve object
	 *
	 * Constructs a RenderParametricCurve object to visualize a Parametric curve.
	 *
	 * @param surface [in] the ParametricCurve to draw
	 */
	RenderParametricCurve(ParametricCurve::Ptr curve);

    /**
     * @brief Destructor
     */
    virtual ~RenderParametricCurve();

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
    rwlibs::nurbs::ParametricCurve::Ptr _curve;
    float _r, _g, _b;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_RENDERPARAMETRICCURVE_HPP_ */
