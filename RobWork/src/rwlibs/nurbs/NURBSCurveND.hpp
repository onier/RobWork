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

#ifndef RWLIBS_NURBS_NURBSCURVEND_HPP_
#define RWLIBS_NURBS_NURBSCURVEND_HPP_

/**
 * @file NURBSCurveND.hpp
 *
 * \copydoc rwlibs::nurbs::NURBSCurveND
 */

#include "NUBSCurveND.hpp"
#include "ParametricUtility.hpp"

namespace rwlibs {
namespace nurbs {

template<std::size_t N>
class NUBSCurveND;

//! @addtogroup nurbs
//! @{
template<std::size_t N>
class NURBSCurveND {
public:
	NURBSCurveND(const NURBSCurveND<N> &curve):
		_points(curve.getPoints()),
		_weights(curve.getWeights()),
		_knots(curve.getKnots()),
		_order(curve.getOrder()),
		_homogeneous(new NUBSCurveND<N+1>(*curve._homogeneous)),
		_denominator(NULL),
		_numerator(NULL),
		_derivedFrom(NULL),
		_derivative(NULL)
	{
		if (curve._denominator != NULL)
			_denominator = new NUBSCurveND<1>(*curve._denominator);
		if (curve._numerator != NULL)
			_numerator = new NUBSCurveND<N>(*curve._numerator);
		if (curve._derivative != NULL)
			_derivative = new NURBSCurveND<N>(*curve._derivative);
	}

	NURBSCurveND(const std::vector<rw::math::VectorND<N> > &points = std::vector<rw::math::VectorND<N> >(), const std::vector<double> &weights = std::vector<double>(), const std::vector<double> &knots = std::vector<double>(), std::size_t order = 0):
		_points(points),
		_weights(weights),
		_knots(knots),
		_order(order),
		_homogeneous(new NUBSCurveND<N+1>(ParametricUtility::toHomogeneous<N>(points,weights),knots,order)),
		_denominator(NULL),
		_numerator(NULL),
		_derivedFrom(NULL),
		_derivative(NULL)
	{
		if (_weights.size() == 0) {
			for (std::size_t i = 0; i < points.size(); i++)
				_weights.push_back(1);
		}
	}

	NURBSCurveND(const std::vector<rw::math::VectorND<N+1> > &pointsHomogeneous, const std::vector<double> &knots, std::size_t order):
		_points(ParametricUtility::fromHomogeneous<N>(pointsHomogeneous)),
		_weights(ParametricUtility::getWeights<N>(pointsHomogeneous)),
		_knots(knots),
		_order(order),
		_homogeneous(new NUBSCurveND<N+1>(pointsHomogeneous,knots,order)),
		_denominator(NULL),
		_numerator(NULL),
		_derivedFrom(NULL),
		_derivative(NULL)
	{
	}

	NURBSCurveND(const NUBSCurveND<N+1> &nonrational):
		_points(ParametricUtility::fromHomogeneous<N>(nonrational.getPoints())),
		_weights(ParametricUtility::getWeights<N>(nonrational.getPoints())),
		_knots(nonrational.getKnots()),
		_order(nonrational.getOrder()),
		_homogeneous(new NUBSCurveND<N+1>(nonrational)),
		_denominator(NULL),
		_numerator(NULL),
		_derivedFrom(NULL),
		_derivative(NULL)
	{
	}

	NURBSCurveND(const NUBSCurveND<N> &curve):
		_points(curve.getPoints()),
		_weights(std::vector<double>()),
		_knots(curve.getKnots()),
		_order(curve.getOrder()),
		_homogeneous(new NUBSCurveND<N+1>(ParametricUtility::toHomogeneous<N>(curve.getPoints()),curve._knots,curve._order)),
		_denominator(NULL),
		_numerator(NULL),
		_derivedFrom(NULL),
		_derivative(NULL)
	{
		for (std::size_t i = 0; i < curve.getPoints().size(); i++)
			_weights.push_back(1);
	}

	~NURBSCurveND() {
		delete _homogeneous;
		if (_denominator != NULL)
			delete _denominator;
		if (_numerator != NULL)
			delete _numerator;
	}

	std::vector<rw::math::VectorND<N> > getPoints() const {
		validate();
		return _points;
	}

	std::vector<rw::math::VectorND<N+1> > getPointsHomogeneous() const {
		validate();
		return _homogeneous->getPoints();
	}

	std::vector<double> getWeights() const {
		validate();
		return _weights;
	}

	std::vector<double> getKnots() const {
		validate();
		return _knots;
	}

	std::size_t getDegree() const {
		validate();
		return _order-1;
	}

	std::size_t getOrder() const {
		validate();
		return _order;
	}

	rw::math::VectorND<N> getValue(double t) const {
		return ParametricUtility::fromHomogeneous<N>(_homogeneous->getValue(t));
	}

	NURBSCurveND<N>& operator=(const NURBSCurveND<N> &rhs) {
		_points = rhs.getPoints();
		_weights = rhs.getWeights();
		_knots = rhs.getKnots();
		_order = rhs.getOrder();
		_homogeneous =  new NUBSCurveND<N+1>(*rhs._homogeneous);
		if (rhs._denominator != NULL)
			_denominator = new NUBSCurveND<1>(*rhs._denominator);
		if (rhs._numerator != NULL)
			_numerator = new NUBSCurveND<N>(*rhs._numerator);
		if (rhs._derivative != NULL)
			_derivative = new NURBSCurveND<N>(*rhs._derivative);
		return *this;
	}

private:
	void validate() const {
		if (_order < 2) {
			std::stringstream str;
			str << "NURBS Curve is not valid (order should be at least 2 - not " << _order << ")";
			RW_THROW(str.str());
		}
		if (_points.size() < _order) {
			std::stringstream str;
			str << "NURBS Curve is not valid (at least " << _order << " control points needed)";
			RW_THROW(str.str());
		}
		if (_knots.size() != _points.size()+_order) {
			std::stringstream str;
			str << "NURBS Curve is not valid (there should be " << (_points.size()+_order) << " knots, but there is " << _knots.size() << ")";
			RW_THROW(str.str());
		}
		if (_weights.size() != _points.size()) {
			std::stringstream str;
			str << "NURBS Curve is not valid (there must be an equal number of weights and control points)";
			RW_THROW(str.str());
		}
	}

	std::vector<rw::math::VectorND<N> > _points;
	std::vector<double> _weights;
	std::vector<double> _knots;
	std::size_t _order;

	NUBSCurveND<N+1>* _homogeneous;
	NUBSCurveND<1>* _denominator;
	NUBSCurveND<N>* _numerator;
	NURBSCurveND<N>* _derivedFrom;
	NURBSCurveND<N>* _derivative;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_NURBSCURVEND_HPP_ */
