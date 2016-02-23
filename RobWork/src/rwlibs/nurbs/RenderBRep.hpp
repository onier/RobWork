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

#ifndef RWLIBS_NURBS_RENDERBREP_HPP_
#define RWLIBS_NURBS_RENDERBREP_HPP_

/**
 * @file RenderBRep.hpp
 *
 * \copydoc rwlibs::nurbs::RenderBRep
 */

#include <list>

#include <rw/graphics/Render.hpp>

#include "BRep.hpp"

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class RenderBRep: public rw::graphics::Render {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<RenderBRep> Ptr;

	/**
	 * @brief Constructs a RenderBRep object
	 *
	 * Constructs a RenderBRep object to visualize a BRep surface.
	 *
	 * @param brep [in] the BRep to draw
	 */
	RenderBRep(BRep::Ptr brep);

	/**
	 * @brief Destructor
	 */
	virtual ~RenderBRep();

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

private:
	std::list<rw::graphics::Render*> _faceRenders;
	std::list<rw::graphics::Render*> _edgeRenders;
	std::list<rw::graphics::Render*> _vertexRenders;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_RENDERBREP_HPP_ */
