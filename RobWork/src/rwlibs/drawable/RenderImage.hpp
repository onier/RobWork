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


#ifndef RWLIBS_DRAWABLE_RENDERIMAGE_HPP
#define RWLIBS_DRAWABLE_RENDERIMAGE_HPP

/**
 * @file RenderFrame.hpp
 */

#include <rwlibs/os/rwgl.hpp>
#include <rw/sensor/Image.hpp>
#include <rw/common/Ptr.hpp>
#include "RWGLTexture.hpp"
#include "Render.hpp"

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief RenderImage renders a image in a plane defined by
     * [-w/2;h/2][w/2;-h/2]. The image can be scaled
     */
    class RenderImage : public Render
    {
    public:
    	RenderImage(float scale=1.0/1000.0);
        /**
         * @brief Constructs a RenderFrame
         * @param size [in] size of the frame coordinate system
         */
        RenderImage(const rw::sensor::Image& img, float scale=1.0/1000.0);

        /**
         * @brief Destructor
         */
        virtual ~RenderImage(){};

    	/* Functions inherited from Render */

        void setImage(const rw::sensor::Image& img);

        /**
         * @copydoc Render::draw
         */
        void draw(DrawType type, double alpha) const;

    private:
        int _w, _h;
        float _scale;
        RWGLTexture _tex;
    };

    typedef rw::common::Ptr<RenderImage> RenderImagePtr;

    /*@}*/
}} // end namespaces

#endif // end include guard
