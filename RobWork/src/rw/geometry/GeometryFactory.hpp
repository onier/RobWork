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


#ifndef RW_GEOMETRY_GEOMETRYFACTORY_HPP_
#define RW_GEOMETRY_GEOMETRYFACTORY_HPP_

#include <rw/common/Cache.hpp>
#include "Geometry.hpp"
#include "GeometryData.hpp"

//! @file GeometryFactory.hpp

namespace rw { namespace geometry {


    /** @addtogroup geometry */
    /*@{*/

    /**
     * @brief Factory for geometric primitive
     *
     * The following primitives are supported
     *
     * Box:
     * Syntax: "#Box dx dy dz"
     * where "dx, dy, dz" are floats specifying the dimensions
     *
     * Cylinder:
     * Syntax: "#Cylinder radius height level"
     * where "radius" and "height" are float specifying radius and
     * height and "level" is a non-negative integer specifying the
     * discretization level.
     *
     * Sphere:
     * Syntax: "#Sphere radi"
     * where radi is the radius of the sphere
     *
     *
     */
    class GeometryFactory
    {
    public:
        /**
         * @brief Factory method for geometric primitive
         *
         * The Factory takes no ownership of the Geometry objects it constructs
         * and thus returns the geometry by auto_ptr.
         *
         * An exception is thrown if the string cannot be parsed correctly.
         *
         * @param str [in] string to parse
         * @return Pointer to a new geometry object
         */
    	static GeometryPtr load(const std::string& str, bool useCache=true);

    	//! @copydoc load
    	static GeometryPtr getGeometry(const std::string& str, bool useCache=true);

        /**
         * @brief loads collision geometry as specified by properties on the frame.
         *
         * Each CollisionModelInfo that is in the PropertyMap of the frame \b f is
         * loaded and returned.
         * @param f [in] the frame where the properties are specified
         * @return a vector of all geometries that was successfully loaded
         */
        static std::vector<GeometryPtr> loadCollisionGeometry(const rw::kinematics::Frame &f);

    private:
        typedef rw::common::Cache<std::string, GeometryData> Cache;
        static Cache& getCache();

    };

    /* @} */


}
} // end namespaces


#endif /* GEOMETRYFACTORY_HPP_ */
