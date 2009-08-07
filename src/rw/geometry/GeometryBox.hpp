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


#ifndef RW_GEOMETRY_GEOMETRYBOX_HPP
#define RW_GEOMETRY_GEOMETRYBOX_HPP

#include "Geometry.hpp"

namespace rw { namespace geometry {

    /** @addtogroup geometry */
    /*@{*/

    /**
     * @brief Provides a geometric box primitive
     *
     * GeometryBox provides a triangulated box primitive. The centre
     * of any box will be \f$(0,0,0\f$, it will be aligned to the axis
     * and its dimensions as specified in the constructor.
     */
    class GeometryBox: public Geometry {
    public:
        /**
         * @brief Constructs box with the specified dimensions
         *
         * The box axis aligned and goes from
         * \f$(-\frac{dx}{2},-\frac{dy}{2},-\frac{dz}{2})\f$
         * to \f$(\frac{dx}{2},\frac{dy}{2},\frac{dz}{2})\f$.
         *
         * @param dx [in] x size
         * @param dy [in] y size
         * @param dz [in] z size;
         */
        GeometryBox(float dx, float dy, float dz);

        /**
         * @brief Destructor
         */
        virtual ~GeometryBox();

        /**
         * @copydoc Geometry::getFaces
         */
        virtual const std::vector<Face<float> >& getFaces() const;
    private:
        std::vector<Face<float> > _faces;

    };

    /*@}*/

}} // end namespaces

#endif // end include guard
