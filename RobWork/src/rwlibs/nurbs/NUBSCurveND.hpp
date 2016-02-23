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

#ifndef RWLIBS_NURBS_NUBSCURVEND_HPP_
#define RWLIBS_NURBS_NUBSCURVEND_HPP_

/**
 * @file NUBSCurveND.hpp
 *
 * \copydoc rwlibs::nurbs::NUBSCurveND
 */

#include "BezierCurveND.hpp"

namespace rwlibs {
namespace nurbs {

template<std::size_t N>
class NURBSCurveND;

//! @addtogroup nurbs
//! @{
template<std::size_t N>
class NUBSCurveND {
public:
	NUBSCurveND(const NUBSCurveND<N> &curve):
		_points(curve._points),
		_knots(curve._knots),
		_order(curve._order)
	{
	}

	NUBSCurveND(const std::vector<rw::math::VectorND<N> > &points = std::vector<rw::math::VectorND<N> >(), const std::vector<double> &knots = std::vector<double>(), std::size_t order = 0):
		_points(points),
		_knots(knots),
		_order(order)
	{
	}

	NUBSCurveND(const NURBSCurveND<N-1> &rational):
		_points(ParametricUtility::toHomogeneous<N-1>(rational.getPoints(),rational.getWeights())),
		_knots(rational._knots),
		_order(rational._order)
	{
	}

	virtual ~NUBSCurveND() {
	}

	std::vector<rw::math::VectorND<N> > getPoints() const {
		validate();
		return _points;
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
		return deBoor(t);
	}
/*
	std::vector<BezierCurveND<N> > decompose() {

	}*/

	std::size_t getMultiplicity(std::size_t k, double tol = 0.) const {
		std::size_t res = 0;
		for (int i = k; i < (int)_knots.size(); i++) {
			if (_knots[i]-_knots[k] <= tol)
				res++;
			else
				break;
		}
		for (int i = k-1; i >= 0; i--) {
			if (_knots[k]-_knots[i] <= tol)
				res++;
			else
				break;
		}
		return res;
	}

private:
	rw::math::VectorND<N> deBoor(double t) const {
		rw::math::VectorND<N> res;
		std::size_t k = 0;
		for (std::size_t i = 1; i < _knots.size()-1; i++) {
			if (t < _knots[i+1])
				k = i;
		}
		std::size_t p = getDegree();
		std::size_t h = p;
		std::size_t s = getMultiplicity(k);
		if (t == _knots[k])
			h = getDegree()-s;
		std::vector<rw::math::VectorND<N> > points;
		for (std::size_t i = k-p; i <= k-s; i++) {
			points.push_back(_points[i]);
		}
		for (std::size_t r = 1; r <= h; r++) {
			std::vector<rw::math::VectorND<N> > newPoints;
			for (std::size_t i = r; i <= p-s; i++) {
				double a = (t-_knots[i+k-p])/(_knots[i+k-r+1]-_knots[i+k-p]);
				rw::math::VectorND<N> P = (1-a)*points[i-1]+a*points[i];
				if (i == p-s && r == p-s)
					res = P;
				newPoints.push_back(P);
			}
			points = newPoints;
		}
		return res;
	}

	void validate() const {
		if (_order < 2) {
			std::stringstream str;
			str << "NUBS Curve is not valid (order should be at least 2 - not " << _order << ")";
			RW_THROW(str.str());
		}
		if (_points.size() < _order) {
			std::stringstream str;
			str << "NUBS Curve is not valid (at least " << _order << " control points needed)";
			RW_THROW(str.str());
		}
		if (_knots.size() != _points.size()+_order) {
			std::stringstream str;
			str << "NUBS Curve is not valid (there should be " << (_points.size()+_order) << " knots, but there is " << _knots.size() << ")";
			RW_THROW(str.str());
		}
	}

	std::vector<rw::math::VectorND<N> > _points;
	std::vector<double> _knots;
	std::size_t _order;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_NUBSCURVEND_HPP_ */
