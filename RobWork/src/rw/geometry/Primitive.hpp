/*
 * Primitive.hpp
 *
 *  Created on: 29/01/2010
 *      Author: jimali
 */

#ifndef RW_GEOMETRY_PRIMITIVE_HPP_
#define RW_GEOMETRY_PRIMITIVE_HPP_

#include "GeometryData.hpp"
#include "TriMesh.hpp"
#include <rw/math/Q.hpp>

//! @file Box.hpp

namespace rw {
namespace geometry {

    //! @addtogroup geometry @{

    /**
     * @brief defines an interface for a geometric shape that is defined
     * by a set of parameters.
     */
    class Primitive: public GeometryData {
    public:

    	/**
    	 * @copydoc GeometryData::getTriMesh
    	 * @note primitives allways return a new trimesh
    	 */
    	TriMeshPtr getTriMesh(bool forceCopy=true){
    		return createMesh(20);
    	}

    	/**
    	 * @brief make a trimesh from this primitive. Use \b granularity to
    	 * specify minimum number of line segments a half circle is split into
    	 * @param resolution [in]
    	 */
        virtual TriMeshPtr createMesh(int resolution) const = 0;

        /**
         * @brief the set of parameters that defines this primitive
         */
        virtual rw::math::Q getParameters() const = 0;

    };
    //! @}
}
}

#endif /* PRIMITIVE_HPP_ */
