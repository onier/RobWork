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

#ifndef RWLIBS_NURBS_NURBSSURFACEND_HPP_
#define RWLIBS_NURBS_NURBSSURFACEND_HPP_

/**
 * @file NURBSSurfaceND.hpp
 *
 * \copydoc rwlibs::nurbs::NURBSSurfaceND
 */

#include "NUBSSurfaceND.hpp"
#include "ParametricUtility.hpp"

namespace rwlibs {
namespace nurbs {

template<std::size_t N>
class NUBSSurfaceND;

//! @addtogroup nurbs
//! @{
template<std::size_t N>
class NURBSSurfaceND {
public:
	typedef std::vector<std::vector<rw::math::VectorND<N> > > PointMap;
	typedef std::vector<std::vector<rw::math::VectorND<N+1> > > PointMapHomogeneous;
	typedef std::vector<std::vector<double> > WeightMap;

	NURBSSurfaceND(const NURBSSurfaceND<N> &surface):
		_points(surface._points),
		_weights(surface._weights),
		_knotsU(surface._knotsU),
		_knotsV(surface._knotsV),
		_orderU(surface._orderU),
		_orderV(surface._orderV),
		_homogeneous(new NUBSSurfaceND<N+1>(*surface._homogeneous)),
		_denominator(NULL),
		_numerator(NULL)
	{
		if (surface._denominator != NULL)
			_denominator = new NUBSSurfaceND<1>(*surface._denominator);
		if (surface._numerator != NULL)
			_numerator = new NUBSSurfaceND<N>(*surface._numerator);
	}

	NURBSSurfaceND(
			const PointMap &points = PointMap(),
			const WeightMap &weights = WeightMap(),
			const std::vector<double> &knotsU = std::vector<double>(),
			const std::vector<double> &knotsV = std::vector<double>(),
			std::size_t orderU = 0,
			std::size_t orderV = 0
	):
		_points(points),
		_weights(weights),
		_knotsU(knotsU),
		_knotsV(knotsV),
		_orderU(orderU),
		_orderV(orderV),
		_homogeneous(new NUBSSurfaceND<N+1>(ParametricUtility::toHomogeneous<N>(points,weights),knotsU,knotsV,orderU,orderV)),
		_denominator(NULL),
		_numerator(NULL)
	{
		if (_weights.size() == 0) {
			for (std::size_t i = 0; i < points.size(); i++) {
				std::vector<double> w;
				for (std::size_t j = 0; j < points[0].size(); j++)
					w.push_back(1);
				_weights.push_back(w);
			}
		}
	}

	NURBSSurfaceND(
			const PointMapHomogeneous &pointsHomogeneous,
			const std::vector<double> &knotsU = std::vector<double>(),
			const std::vector<double> &knotsV = std::vector<double>(),
			std::size_t orderU = 0,
			std::size_t orderV = 0
	):
		_points(ParametricUtility::fromHomogeneous<N>(pointsHomogeneous)),
		_weights(ParametricUtility::getWeights<N>(pointsHomogeneous)),
		_knotsU(knotsU),
		_knotsV(knotsV),
		_orderU(orderU),
		_orderV(orderV),
		_homogeneous(new NUBSSurfaceND<N+1>(pointsHomogeneous,knotsU,knotsV,orderU,orderV)),
		_denominator(NULL),
		_numerator(NULL)
	{
		if (_weights.size() == 0) {
			for (std::size_t i = 0; i < _points.size(); i++) {
				std::vector<double> w;
				for (std::size_t j = 0; j < _points[0].size(); j++)
					w.push_back(1);
				_weights.push_back(w);
			}
		}
	}

	NURBSSurfaceND(const NUBSSurfaceND<N+1> &nonrational):
		_points(ParametricUtility::fromHomogeneous<N>(nonrational.getPoints())),
		_weights(ParametricUtility::getWeights<N>(nonrational.getPoints())),
		_knotsU(nonrational.getKnotsU()),
		_knotsV(nonrational.getKnotsV()),
		_orderU(nonrational.getOrderU()),
		_orderV(nonrational.getOrderV()),
		_homogeneous(new NUBSSurfaceND<N+1>(nonrational)),
		_denominator(NULL),
		_numerator(NULL)
	{
	}

	NURBSSurfaceND(const NUBSSurfaceND<N> &surface):
		_points(surface.getPoints()),
		_weights(std::vector<double>()),
		_knotsU(surface.getKnotsU()),
		_knotsV(surface.getKnotsV()),
		_orderU(surface.getOrderU()),
		_orderV(surface.getOrderV()),
		_homogeneous(new NUBSSurfaceND<N+1>(ParametricUtility::toHomogeneous<N>(surface.getPoints()),surface._knotsU,surface._knotsV,surface._orderU,surface._orderV)),
		_denominator(NULL),
		_numerator(NULL)
	{
		for (std::size_t i = 0; i < surface.getPoints().size(); i++) {
			std::vector<double> w;
			for (std::size_t j = 0; j < surface.getPoints()[0].size(); j++)
				w.push_back(1);
			_weights.push_back(w);
		}
	}

	virtual ~NURBSSurfaceND() {
		if (_denominator != NULL)
			delete _denominator;
		if (_numerator != NULL)
			delete _numerator;
	}

	PointMap getPoints() const {
		validate();
		return _points;
	}

	PointMapHomogeneous getPointsHomogeneous() const {
		validate();
		return _homogeneous->getPoints();
	}

	WeightMap getWeights() const {
		validate();
		return _weights;
	}

	std::vector<double> getKnotsU() const {
		validate();
		return _knotsU;
	}

	std::vector<double> getKnotsV() const {
		validate();
		return _knotsV;
	}

	std::size_t getDegreeU() const {
		validate();
		return _orderU-1;
	}

	std::size_t getDegreeV() const {
		validate();
		return _orderV-1;
	}

	std::size_t getOrderU() const {
		validate();
		return _orderU;
	}

	std::size_t getOrderV() const {
		validate();
		return _orderV;
	}

	rw::math::VectorND<N> getValue(double u, double v) const {
		return ParametricUtility::fromHomogeneous<N>(_homogeneous->getValue(u,v));
	}

	std::size_t getMultiplicityU(std::size_t k, double tol = 0.) const {
		std::size_t res = 0;
		for (int i = k; i < (int)_knotsU.size(); i++) {
			if (_knotsU[i]-_knotsU[k] <= tol)
				res++;
			else
				break;
		}
		for (int i = k-1; i >= 0; i--) {
			if (_knotsU[k]-_knotsU[i] <= tol)
				res++;
			else
				break;
		}
		return res;
	}

	std::size_t getMultiplicityV(std::size_t k, double tol = 0.) const {
		std::size_t res = 0;
		for (int i = k; i < (int)_knotsV.size(); i++) {
			if (_knotsV[i]-_knotsV[k] <= tol)
				res++;
			else
				break;
		}
		for (int i = k-1; i >= 0; i--) {
			if (_knotsV[k]-_knotsV[i] <= tol)
				res++;
			else
				break;
		}
		return res;
	}

private:
	void validate() const {
		if (_orderU < 2) {
			std::stringstream str;
			str << "NUBS Curve is not valid (order should be at least 2 in U-direction - not " << _orderU << ")";
			RW_THROW(str.str());
		}
		if (_orderV < 2) {
			std::stringstream str;
			str << "NUBS Curve is not valid (order should be at least 2 in V-direction - not " << _orderV << ")";
			RW_THROW(str.str());
		}
		if (_points.size() < _orderU) {
			std::stringstream str;
			str << "NUBS Curve is not valid (at least " << _orderU << " control points needed in U-direction)";
			RW_THROW(str.str());
		}
		if (_points[0].size() < _orderV) {
			std::stringstream str;
			str << "NUBS Curve is not valid (at least " << _orderV << " control points needed in V-direction)";
			RW_THROW(str.str());
		}
		if (_knotsU.size() != _points.size()+_orderU) {
			std::stringstream str;
			str << "NUBS Curve is not valid (there should be " << (_points.size()+_orderU) << " knots, but there is " << _knotsU.size() << " in U-direction)";
			RW_THROW(str.str());
		}
		if (_knotsV.size() != _points[0].size()+_orderV) {
			std::stringstream str;
			str << "NUBS Curve is not valid (there should be " << (_points[0].size()+_orderV) << " knots, but there is " << _knotsV.size() << " in V-direction)";
			RW_THROW(str.str());
		}
		if (_weights.size() != _points.size()) {
			std::stringstream str;
			str << "NURBS Curve is not valid (there must be an equal number of weights and control points in U-direction)";
			RW_THROW(str.str());
		}
		if (_weights[0].size() != _points[0].size()) {
			std::stringstream str;
			str << "NURBS Curve is not valid (there must be an equal number of weights and control points in V-direction)";
			RW_THROW(str.str());
		}
	}

	PointMap _points;
	WeightMap _weights;
	std::vector<double> _knotsU, _knotsV;
	std::size_t _orderU, _orderV;

	NUBSSurfaceND<N+1>* _homogeneous;
	NUBSSurfaceND<1>* _denominator;
	NUBSSurfaceND<N>* _numerator;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_NURBSSURFACEND_HPP_ */
