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


#ifndef rwlibs_drawable_RenderModel3D_HPP
#define rwlibs_drawable_RenderModel3D_HPP

/**
 * @file Render3DS.hpp
 */

#include "Model3D.hpp"
#include "Render.hpp"

#include <rwlibs/os/rwgl.hpp>

#include <cstring>
#include <iostream>

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief This class loads 3d scenes or objects from a 3ds file
     * format.
     *
     */
    class RenderModel3D : public Render {
    private:
        Model3DPtr _model;

    public:
        /**
         * @brief creates a Render3DS given a 3DS file.
         * @param filename [in] - the path and name of the 3DS file
         */
        RenderModel3D(Model3DPtr model);

        /**
         * @brief Destructor
         */
    	virtual ~RenderModel3D(){}

    	// Functions inherited from Render

        /**
         * @copydoc Render::draw
         */
        void draw(DrawType type, double alpha) const;

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
