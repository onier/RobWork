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

#ifndef RWLIBS_NURBS_NUBSSURFACEND_HPP_
#define RWLIBS_NURBS_NUBSSURFACEND_HPP_

/**
 * @file NUBSSurfaceND.hpp
 *
 * \copydoc rwlibs::nurbs::NUBSSurfaceND
 */

#include "ParametricUtility.hpp"

namespace rwlibs {
namespace nurbs {

template<std::size_t N>
class NURBSSurfaceND;

//! @addtogroup nurbs
//! @{
template<std::size_t N>
class NUBSSurfaceND {
public:
	typedef std::vector<std::vector<rw::math::VectorND<N> > > PointMap;

	NUBSSurfaceND(const NUBSSurfaceND<N> &surface):
		_points(surface._points),
		_knotsU(surface._knotsU),
		_knotsV(surface._knotsV),
		_orderU(surface._orderU),
		_orderV(surface._orderV)
	{
	}

	NUBSSurfaceND(
			const PointMap &points = PointMap(),
			const std::vector<double> &knotsU = std::vector<double>(),
			const std::vector<double> &knotsV = std::vector<double>(),
			std::size_t orderU = 0,
			std::size_t orderV = 0
			):
		_points(points),
		_knotsU(knotsU),
		_knotsV(knotsV),
		_orderU(orderU),
		_orderV(orderV)
	{
	}

	NUBSSurfaceND(const NURBSSurfaceND<N-1> &rational):
		_points(ParametricUtility::toHomogeneous<N-1>(rational.getPoints(),rational.getWeights())),
		_knotsU(rational._knotsU),
		_knotsV(rational._knotsV),
		_orderU(rational._orderU),
		_orderV(rational._orderV)
	{
	}

	virtual ~NUBSSurfaceND() {
	}

	PointMap getPoints() const {
		validate();
		return _points;
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
		return rw::math::VectorND<N>::zero();
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
	}

	PointMap _points;
	std::vector<double> _knotsU, _knotsV;
	std::size_t _orderU, _orderV;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_NUBSSURFACEND_HPP_ */
