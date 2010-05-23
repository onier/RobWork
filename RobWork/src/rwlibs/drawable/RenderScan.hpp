/*
 * RenderScan.hpp
 *
 *  Created on: 23/05/2010
 *      Author: jimali
 */

#ifndef RWLIBS_DRAWABLE_RENDERSCAN_HPP_
#define RWLIBS_DRAWABLE_RENDERSCAN_HPP_

#include <rwlibs/os/rwgl.hpp>
#include <rw/sensor/Image25D.hpp>
#include <rw/sensor/Scan2D.hpp>
#include "RWGLTexture.hpp"
#include "Render.hpp"

namespace rwlibs {
namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief RenderImage renders a image in a plane defined by
     * [-w/2;h/2][w/2;-h/2]. The image can be scaled
     */
	class RenderScan: public Render{
	public:
		RenderScan();
		virtual ~RenderScan();

		/**
		 * \brief set a 2.5 dimensional scan
		 * @param img
		 */
		void setScan(const rw::sensor::Image25D& img);

		/**
		 * \brief set a two dimensional scan
		 * @param img
		 */
		void setScan(const rw::sensor::Scan2D& img);

		/**
		 * \brief set a one dimensional scan
		 * @param dist
		 */
		void setScan(float dist);

        /**
         * @copydoc Render::draw
         */
        void draw(DrawType type, double alpha) const;

        void setMinDepth(float depth){ _minDepth=depth;};
        void setMaxDepth(float depth){ _maxDepth=depth;};

	private:
        rw::sensor::Image25D _img;
        float _minDepth,_maxDepth;
	};
}
}
#endif /* RENDERSCAN_HPP_ */
