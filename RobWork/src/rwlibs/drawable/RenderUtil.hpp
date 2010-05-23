/*
 * RenderUtil.hpp
 *
 *  Created on: 23/05/2010
 *      Author: jimali
 */

#ifndef RENDERUTIL_HPP_
#define RENDERUTIL_HPP_

#include "RenderLines.hpp"

namespace rwlibs {
namespace drawable {
/**
 * @brief collection of utillities for rendering
 */
class RenderUtil {
public:

	/**
	 * @brief create a camera view render for a pinhole camera.
	 * The camera looks in the negative direction of the z-axis.
	 *
	 * @param w [in] focal width
	 * @param h [in] focal height
	 * @param fovy [in] horisontal field of view in degree
	 * @param near [in] near clipping plane
	 * @param far [in] far clipping plane
	 * @return render that renders the camera view with lines
	 */
	static rw::common::Ptr<RenderLines> makeCameraViewRender(
			double w, double h, double fovy,
			double near=0, double far=2.0);


	/**
	 * @brief creates a rectangular grid of size \b size with
	 * \b resolution as the size of each grid mask
	 * @param size
	 * @param resolution
	 * @return render that renders a rectangular grid
	 */
	static rw::common::Ptr<RenderLines> makeWorldGridRender(
			float size, float resolution);

};
}
}
#endif /* RENDERUTIL_HPP_ */
