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

#ifndef RWLIBS_NURBS_NURBSCURVE_HPP_
#define RWLIBS_NURBS_NURBSCURVE_HPP_

/**
 * @file NURBSCurve.hpp
 *
 * \copydoc rwlibs::nurbs::NURBSCurve
 */

#include "ParametricCurve.hpp"

#include <vector>

#include <rw/math/Vector3D.hpp>

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class NURBSCurve: public rwlibs::nurbs::ParametricCurve {
public:
    //! @brief smart pointer type to this class
	typedef rw::common::Ptr<NURBSCurve> Ptr;

	/**
	 * @copydoc rwlibs::nurbs::ParametricCurve::getTriMesh
	 */
	virtual rw::common::Ptr<rw::geometry::TriMesh> getTriMesh(bool forceCopy=true);

	/**
	 * @copydoc rwlibs::nurbs::ParametricCurve::isConvex
	 */
	virtual bool isConvex();

	/**
	 * @brief Construct new NURBSCurve.
	 * @param order the order.
	 * @param controlPoints a list of controlpoints.
	 * @param knots the knots.
	 * @param weights the weights.
	 */
	NURBSCurve(
			unsigned int order,
			std::vector<rw::math::Vector3D<> > controlPoints,
			std::vector<double> knots,
			std::vector<double> weights);

	/**
	 * @brief destructor
	 */
	virtual ~NURBSCurve();

	/**
	 * @brief Get the degree of the NURBS curve.
	 * @return the degree.
	 */
	std::size_t getDegree() const;

	/**
	 * @brief Get the order of the NURBS curve.
	 * @return the order.
	 */
	std::size_t getOrder() const;

	/**
	 * @brief Get the controlpoints of the NURBS curve.
	 * @return the controlpoints.
	 */
	std::vector<rw::math::Vector3D<> > getControlPoints() const;

	/**
	 * @brief Get the knots.
	 * @return the knots.
	 */
	std::vector<double> getKnots() const;

	/**
	 * @brief Get the full knot vector.
	 * @return the full list of knots.
	 */
	std::vector<double> getFullKnots() const;

	/**
	 * @brief Get the weights of the curve.
	 * @return the weights.
	 */
	std::vector<double> getWeights() const;

	rw::math::Vector3D<> evaluate(double t);
	rw::math::Vector3D<> evaluateDerivative(double t, unsigned int times = 1);

	static NURBSCurve* makeCircle(rw::math::Vector3D<> center, rw::math::Vector3D<> normal, double radius);

private:
	static double B(unsigned int i, unsigned int n, std::vector<double> knots, double t);
	static rw::math::Vector3D<> getPerpendicular(rw::math::Vector3D<> vec);

	std::size_t _order;
	std::vector<rw::math::Vector3D<> > _controlPoints;
	std::vector<double> _knots;
	std::vector<double> _weights;
	NURBSCurve* _derivative;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_NURBSCURVE_HPP_ */
