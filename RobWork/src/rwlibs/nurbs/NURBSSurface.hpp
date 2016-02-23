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

#ifndef RWLIBS_NURBS_NURBSSURFACE_HPP_
#define RWLIBS_NURBS_NURBSSURFACE_HPP_

/**
 * @file NURBSSurface.hpp
 *
 * \copydoc rwlibs::nurbs::NURBSSurface
 */

#include "ParametricSurface.hpp"

#include <rw/geometry/Geometry.hpp>

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class NURBSSurface: public rwlibs::nurbs::ParametricSurface {
public:
    //! @brief smart pointer type to this class
	typedef rw::common::Ptr<NURBSSurface> Ptr;

	typedef std::pair<NURBSSurface::Ptr, NURBSSurface::Ptr> NURBSSurfacePair;

	/**
	 * @copydoc rwlibs::nurbs::ParametricSurface::isConvex
	 */
	virtual bool isConvex();

	/**
	 * @copydoc rwlibs::nurbs::ParametricSurface::evaluate
	 */
	virtual rw::math::Vector3D<> evaluate(double u, double v);

	/**
	 * @copydoc rwlibs::nurbs::ParametricSurface::normal
	 */
	virtual rw::math::Vector3D<> normal(double u, double v);

	/**
	 * @brief Construct new NURBSSurface.
	 * @param orderU the order in the first direction.
	 * @param orderV the order in the second direction.
	 * @param controlPoints a 2D map of the controlpoints.
	 * @param knotsU the knots in the first direction.
	 * @param knotsV the knots in the second direction.
	 * @param weights the weights.
	 */
	NURBSSurface(
			unsigned int orderU,
			unsigned int orderV,
			std::vector<std::vector<rw::math::Vector3D<> > > controlPoints,
			std::vector<double> knotsU,
			std::vector<double> knotsV,
			std::vector<std::vector<double> > weights);

	/**
	 * @brief Create a clone of a surface.
	 * @param surface the surface to clone.
	 */
	NURBSSurface(const NURBSSurface &surface);

	/**
	 * @brief destructor
	 */
	virtual ~NURBSSurface();

	/**
	 * @brief Get the degree of the NURBS surface.
	 * @return the degree in the two directions as a pair.
	 */
	std::pair<std::size_t, std::size_t> getDegree() const;

	/**
	 * @brief Get the order of the NURBS surface.
	 * @return the order in the two directions as a pair.
	 */
	std::pair<std::size_t, std::size_t> getOrder() const;

	/**
	 * @brief Get the controlpoints of the NURBS surface.
	 * @return the controlpoints in the two dimensions.
	 */
	std::vector<std::vector<rw::math::Vector3D<> > > getControlPoints() const;

	/**
	 * @brief Get the knots.
	 * @return the knots in each direction as a pair of lists.
	 */
	std::pair<std::vector<double>, std::vector<double> > getKnots() const;

	/**
	 * @brief Get the full knot vectors.
	 * @return the full lists of knots.
	 */
	std::pair<std::vector<double>, std::vector<double> > getFullKnots() const;

	/**
	 * @brief Get the weights of the surface.
	 * @return the weights in each direction as a pair of lists.
	 */
	std::vector<std::vector<double> > getWeights() const;

	NURBSSurfacePair divideU(double knot) const;
	NURBSSurfacePair divideV(double knot) const;

	unsigned int getMultiplicityU(unsigned int knotIndex) const;
	unsigned int getMultiplicityV(unsigned int knotIndex) const;

	void insertKnotU(double val);
	void insertKnotV(double val);

	std::vector<rw::math::Vector3D<> > getDerivativeBoundsU() const;
	std::vector<rw::math::Vector3D<> > getDerivativeBoundsV() const;

	rw::geometry::Geometry::Ptr getBoundingSphere() const;
	std::pair<rw::math::Vector3D<>, double> getTangentConeU() const;


	NURBSSurface::Ptr elevate() const;

private:
	static double B(unsigned int i, unsigned int n, std::vector<double> knots, double t);
	static unsigned int findKnotIndex(const std::vector<double> &knots, double value);

	std::pair<std::size_t, std::size_t> _order;
	std::vector<std::vector<rw::math::Vector3D<> > > _controlPoints;
	std::pair<std::vector<double>, std::vector<double> > _knots;
	std::vector<std::vector<double> > _weights;

	double _weightRatio;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_NURBSSURFACE_HPP_ */
