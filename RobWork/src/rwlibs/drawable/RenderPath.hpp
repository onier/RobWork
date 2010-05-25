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

#ifndef RWLIBS_DRAWABLE_RENDERPATH_HPP
#define RWLIBS_DRAWABLE_RENDERPATH_HPP

//! @file RenderPath.hpp

#include "Render.hpp"

namespace rwlibs {
namespace drawable {
    //! @addtogroup drawable @{
    /**
     * @brief loads a path from file and renders it.
     *
     * TODO: file loading should be seperated from render. The rendering can be done using
     * RenderLines and a PathLoader should be constructed instead.
     */
    class RenderPath : public Render
    {
    public:
        /**
         * @brief constructor
         * @param filename [in] filename of path file
         */
        RenderPath(const std::string& filename);

        /**
         * @brief destructor
         * @return
         */
        virtual ~RenderPath();

    protected:
        virtual void update(UpdateType type);

    };
    //! @}
} //end namespace drawable
} //end namespace rwlibs

#endif /*RWLIBS_DRAWABLE_RENDERPATH_HPP*/
