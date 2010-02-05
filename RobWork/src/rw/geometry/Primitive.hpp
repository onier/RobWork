/*
 * Primitive.hpp
 *
 *  Created on: 29/01/2010
 *      Author: jimali
 */

#ifndef PRIMITIVE_HPP_
#define PRIMITIVE_HPP_

#include "GeometryData.hpp"
#include "TriMesh.hpp"
#include <rw/math/Q.hpp>

namespace rw {
namespace geometry {

/**
 * @brief defines an interface for a geometric shape that is defined
 * by a set of parameters.
 */
class Primitive: public GeometryData {
public:

	virtual TriMeshPtr createMesh(int resolution) = 0;

	/**
	 * @brief the set of parameters that defines this primitive
	 */
	virtual const rw::math::Q& getParameters() = 0;

};

}
}

#endif /* PRIMITIVE_HPP_ */
