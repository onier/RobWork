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

#ifndef RWLIBS_NURBS_BEZIERSURFACERATIONALND_HPP_
#define RWLIBS_NURBS_BEZIERSURFACERATIONALND_HPP_

/**
 * @file BezierSurfaceRationalND.hpp
 *
 * \copydoc rwlibs::nurbs::BezierSurfaceRationalND
 */

#include "BezierSurfaceND.hpp"
#include "ParametricUtility.hpp"

namespace rwlibs {
namespace nurbs {

template<std::size_t N>
class BezierSurfaceND;

//! @addtogroup nurbs
//! @{
template<std::size_t N>
class BezierSurfaceRationalND {
public:
	typedef std::vector<std::vector<rw::math::VectorND<N> > > PointMap;
	typedef std::vector<std::vector<rw::math::VectorND<N+1> > > PointMapHomogeneous;
	typedef std::vector<std::vector<double> > WeightMap;

	BezierSurfaceRationalND(const BezierSurfaceRationalND<N> &surface):
		_points(surface.getPoints()),
		_weights(surface.getWeights()),
		_homogeneous(new BezierSurfaceND<N+1>(*surface._homogeneous)),
		_denominator(NULL),
		_numerator(NULL)
	{
		if (surface._denominator != NULL)
			_denominator = new BezierSurfaceND<1>(*surface._denominator);
		if (surface._numerator != NULL)
			_numerator = new BezierSurfaceND<N>(*surface._numerator);
	}

	BezierSurfaceRationalND(const PointMap &points = PointMap(), const WeightMap &weights = WeightMap()):
		_points(points),
		_weights(weights),
		_homogeneous(new BezierSurfaceND<N+1>(ParametricUtility::toHomogeneous<N>(points,weights))),
		_denominator(NULL),
		_numerator(NULL)
	{
		if (_weights.size() == 0) {
			for (std::size_t i = 0; i < points.size(); i++) {
				std::vector<double> w;
				for (std::size_t j = 0; j < points[i].size(); j++) {
					w.push_back(1);
				}
				_weights.push_back(w);
			}
		}
	}

	BezierSurfaceRationalND(const PointMapHomogeneous &pointsHomogeneous):
		_points(ParametricUtility::fromHomogeneous<N>(pointsHomogeneous)),
		_weights(ParametricUtility::getWeights<N>(pointsHomogeneous)),
		_homogeneous(new BezierSurfaceND<N+1>(pointsHomogeneous)),
		_denominator(NULL),
		_numerator(NULL)
	{
	}

	BezierSurfaceRationalND(const BezierSurfaceND<N+1> &nonrational):
		_points(ParametricUtility::fromHomogeneous<N>(nonrational.getPoints())),
		_weights(ParametricUtility::getWeights<N>(nonrational.getPoints())),
		_homogeneous(new BezierSurfaceND<N+1>(nonrational)),
		_denominator(NULL),
		_numerator(NULL)
	{
	}

	BezierSurfaceRationalND(const BezierSurfaceND<N> &surface):
		_points(surface.getPoints()),
		_weights(std::vector<double>()),
		_homogeneous(new BezierCurveND<N+1>(ParametricUtility::toHomogeneous<N>(surface.getPoints()))),
		_denominator(NULL),
		_numerator(NULL)
	{
		for (std::size_t i = 0; i < surface.getPoints().size(); i++) {
			std::vector<double> w;
			for (std::size_t j = 0; j < surface.getPoints().size(); j++) {
				w.push_back(1);
			}
			_weights.push_back(w);
		}
	}

	~BezierSurfaceRationalND() {
		delete _homogeneous;
		if (_denominator != NULL)
			delete _denominator;
		if (_numerator != NULL)
			delete _numerator;
	}

	PointMap getPoints() const {
		return _points;
	}

	std::vector<rw::math::VectorND<N> > getPointsU(std::size_t i) const {
		return _points[i];
	}

	std::vector<rw::math::VectorND<N> > getPointsV(std::size_t j) const {
		std::vector<rw::math::VectorND<N> > points;
		for (std::size_t i = 0; i < getOrder().first; i++)
			points.push_back(_points[i][j]);
		return points;
	}

	PointMapHomogeneous getPointsHomogeneous() const {
		return _homogeneous->getPoints();
	}

	WeightMap getWeights() const {
		return _weights;
	}

	std::vector<double> getWeightsU(std::size_t i) const {
		return _weights[i];
	}

	std::vector<double> getWeightsV(std::size_t j) const {
		std::vector<double> weights;
		for (std::size_t i = 0; i < getOrder().first; i++)
			weights.push_back(_weights[i][j]);
		return weights;
	}

	BezierCurveRationalND<N> getCurveU(double u) const {
		std::vector<rw::math::VectorND<N> > points;
		std::vector<double> weights;
		for (std::size_t j = 0; j < getOrder().second; j++) {
			BezierCurveND<1> wCurve(ParametricUtility::toVectorND(getWeightsV(j)));
			weights.push_back(wCurve.getValue(u)[0]);
			BezierCurveRationalND<N> pCurve(getPointsV(j),getWeightsV(j));
			points.push_back(pCurve.getValue(u));
		}
		return BezierCurveRationalND<N>(points,weights);
	}

	BezierCurveRationalND<N> getCurveV(double v) const {
		std::vector<rw::math::VectorND<N> > points;
		std::vector<double> weights;
		for (std::size_t i = 0; i < getOrder().first; i++) {
			BezierCurveND<1> wCurve(ParametricUtility::toVectorND(getWeightsU(i)));
			weights.push_back(wCurve.getValue(v)[0]);
			BezierCurveRationalND<N> pCurve(getPointsU(i),getWeightsU(i));
			points.push_back(pCurve.getValue(v));
		}
		return BezierCurveRationalND<N>(points,weights);
	}

	rw::math::VectorND<N> getValue(double u, double v) const {
		return ParametricUtility::fromHomogeneous<N>(_homogeneous->getValue(u,v));
	}

	std::pair<std::size_t,std::size_t> getDegree() const {
		if (_points.size() < 1)
			RW_THROW("Bezier Surface is not valid (no control points in U-direction)");
		if (_points[0].size() < 1)
			RW_THROW("Bezier Surface is not valid (no control points in V-direction)");
		std::pair<std::size_t,std::size_t> degree;
		degree.first = _points.size()-1;
		degree.second = _points[0].size()-1;
		return degree;
	}

	std::pair<std::size_t,std::size_t> getOrder() const {
		if (_points.size() < 1)
			RW_THROW("Bezier Curve is not valid (no control points)");
		std::pair<std::size_t,std::size_t> order;
		order.first = _points.size();
		order.second = _points[0].size();
		return order;
	}

	BezierSurfaceRationalND<N> getDerivativeCurveU(std::size_t times = 1) {
		if (times < 1)
			return *this;
		BezierSurfaceND<N> num = getNumerator();
		BezierSurfaceND<1> den = getDenominator();
		// Use the Leibniz formula
		std::size_t k = times;
		BezierSurfaceND<1> denPow = den;
		for (std::size_t i = 1; i < k; i++)
			denPow = dot(denPow,den);
		BezierSurfaceND<N> newNum = num.getDerivativeCurveU(k)*denPow;
		for (std::size_t i = 1; i <= k; i++) {
			BezierSurfaceND<1> denPowJ = den;
			for (std::size_t l = 1; l < i-1; l++)
				denPowJ = dot(denPowJ,den);
			BezierSurfaceND<N> X = getDerivativeCurveU(k-i).getNumerator();
			if (i > 1)
				newNum = newNum - ParametricUtility::binomial(k,i)*den.getDerivativeCurveU(i)*X*denPowJ;
			else
				newNum = newNum - ParametricUtility::binomial(k,i)*den.getDerivativeCurveU(i)*X;
		}
		denPow = dot(denPow,den);
		BezierSurfaceRationalND<N> res(newNum/denPow);
		return res;
	}

	BezierSurfaceRationalND<N> getDerivativeCurveV(std::size_t times = 1) {
		if (times < 1)
			return *this;
		BezierSurfaceND<N> num = getNumerator();
		BezierSurfaceND<1> den = getDenominator();
		// Use the Leibniz formula
		std::size_t k = times;
		BezierSurfaceND<1> denPow = den;
		for (std::size_t i = 1; i < k; i++)
			denPow = dot(denPow,den);
		BezierSurfaceND<N> newNum = num.getDerivativeCurveV(k)*denPow;
		for (std::size_t i = 1; i <= k; i++) {
			BezierSurfaceND<1> denPowJ = den;
			for (std::size_t l = 1; l < i-1; l++)
				denPowJ = dot(denPowJ,den);
			BezierSurfaceND<N> X = getDerivativeCurveV(k-i).getNumerator();
			if (i > 1)
				newNum = newNum - ParametricUtility::binomial(k,i)*den.getDerivativeCurveV(i)*X*denPowJ;
			else
				newNum = newNum - ParametricUtility::binomial(k,i)*den.getDerivativeCurveV(i)*X;
		}
		denPow = dot(denPow,den);
		BezierSurfaceRationalND<N> res(newNum/denPow);
		return res;
	}

	BezierSurfaceRationalND<N> getDerivativeCurve(std::size_t timesU = 1, std::size_t timesV = 1) {
		if (timesU < 1 && timesV < 1)
			return *this;
		BezierSurfaceND<N> num = getNumerator();
		BezierSurfaceND<1> den = getDenominator();
		// Use the Leibniz formula
		std::size_t m = timesU;
		std::size_t n = timesV;
		BezierSurfaceND<1> denPow = den;
		for (std::size_t count = 1; count < m+n; count++)
			denPow = dot(denPow,den);
		BezierSurfaceND<N> newNum = num.getDerivativeCurve(m,n)*denPow;
		for (std::size_t k = 1; k <= n; k++) {
			BezierSurfaceND<1> denPowJ = den;
			for (std::size_t count = 1; count < k-1; count++)
				denPowJ = dot(denPowJ,den);
			BezierSurfaceND<N> X = getDerivativeCurve(m,n-k).getNumerator();
			if (k > 1)
				newNum = newNum - ParametricUtility::binomial(n,k)*den.getDerivativeCurve(0,k)*X*denPowJ;
			else
				newNum = newNum - ParametricUtility::binomial(n,k)*den.getDerivativeCurve(0,k)*X;
		}
		for (std::size_t l = 1; l <= m; l++) {
			for (std::size_t k = 0; k <= n; k++) {
				BezierSurfaceND<1> denPowJ = den;
				for (std::size_t count = 1; count < k+l-1; count++)
					denPowJ = dot(denPowJ,den);
				BezierSurfaceND<N> X = getDerivativeCurve(m-l,n-k).getNumerator();
				if (k > 1)
					newNum = newNum - ParametricUtility::binomial(m,l)*ParametricUtility::binomial(n,k)*den.getDerivativeCurve(l,k)*X*denPowJ;
				else
					newNum = newNum - ParametricUtility::binomial(m,l)*ParametricUtility::binomial(n,k)*den.getDerivativeCurve(l,k)*X;
			}
		}
		denPow = dot(denPow,den);
		BezierSurfaceRationalND<N> res(newNum/denPow);
		return res;
	}

	BezierSurfaceRationalND<N>* elevateU(std::size_t times = 1) const {
		return new BezierSurfaceRationalND<N>(_homogeneous->elevateU(times)->getPoints());
	}

	BezierSurfaceRationalND<N>* elevateV(std::size_t times = 1) const {
		return new BezierSurfaceRationalND<N>(_homogeneous->elevateV(times)->getPoints());
	}

	BezierSurfaceRationalND<N>* elevate(std::size_t timesU = 1, std::size_t timesV = 1) const {
		return new BezierSurfaceRationalND<N>(_homogeneous->elevate(timesU,timesV)->getPoints());
	}

	BezierSurfaceRationalND<N>& operator=(const BezierSurfaceRationalND<N> &rhs) {
		_points = rhs.getPoints();
		_weights = rhs.getWeights();
		_homogeneous =  new BezierSurfaceND<N+1>(*rhs._homogeneous);
		if (rhs._denominator != NULL)
			_denominator = new BezierSurfaceND<1>(*rhs._denominator);
		if (rhs._numerator != NULL)
			_numerator = new BezierSurfaceND<N>(*rhs._numerator);
		return *this;
	}

	const BezierSurfaceRationalND<N> operator+( const BezierSurfaceRationalND<N>& b) const
	{
		BezierSurfaceRationalND<N> first = *this;
		BezierSurfaceRationalND<N> second = b;
		BezierSurfaceND<1> den = dot(first.getDenominator(),second.getDenominator());
		BezierSurfaceRationalND<N> res = (first.getNumerator()*second.getDenominator()+second.getNumerator()*first.getDenominator())/den;
		return res;
	}

	const BezierSurfaceRationalND<N> operator-( const BezierSurfaceRationalND<N>& b) const
	{
		BezierSurfaceRationalND<N> first = *this;
		BezierSurfaceRationalND<N> second = b;
		BezierSurfaceND<1> den = dot(first.getDenominator(),second.getDenominator());
		BezierSurfaceRationalND<N> res = (first.getNumerator()*second.getDenominator()-second.getNumerator()*first.getDenominator())/den;
		return res;
	}

	const BezierSurfaceRationalND<N> operator/( const BezierSurfaceND<1>& b)
	{
		return getNumerator()/dot(getDenominator(),b);
	}

	const BezierSurfaceRationalND<N> operator/( BezierSurfaceRationalND<1>& b)
	{
		return (getNumerator()*b.getDenominator())/(getDenominator()*b.getNumerator());
	}

	const BezierSurfaceRationalND<N> operator*( const BezierSurfaceND<1>& b)
	{
		BezierSurfaceRationalND<N> res = (getNumerator()*b)/getDenominator();
		return res;
	}

private:
	BezierSurfaceND<1> getDenominator() {
		if (_denominator == NULL) {
			std::vector<std::vector<rw::math::VectorND<1> > > points;
			for (std::size_t i = 0; i < _weights.size(); i++) {
				std::vector<rw::math::VectorND<1> > row;
				for (std::size_t j = 0; j < _weights[i].size(); j++) {
					rw::math::VectorND<1> w;
					w[0] = _weights[i][j];
					row.push_back(w);
				}
				points.push_back(row);
			}
			_denominator = new BezierSurfaceND<1>(points);
		}
		return *_denominator;
	}

	BezierSurfaceND<N> getNumerator() {
		if (_numerator == NULL) {

			std::vector<std::vector<rw::math::VectorND<N> > > points;
			for (std::size_t i = 0; i < _weights.size(); i++) {
				std::vector<rw::math::VectorND<N> > row;
				for (std::size_t j = 0; j < _weights[i].size(); j++) {
					row.push_back(_weights[i][j]*_points[i][j]);
				}
				points.push_back(row);
			}
			_numerator = new BezierSurfaceND<N>(points);
		}
		return *_numerator;
	}

	PointMap _points;
	WeightMap _weights;

	BezierSurfaceND<N+1>* _homogeneous;
	BezierSurfaceND<1>* _denominator;
	BezierSurfaceND<N>* _numerator;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_BEZIERSURFACERATIONALND_HPP_ */
