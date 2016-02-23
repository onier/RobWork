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

#ifndef RWLIBS_NURBS_BEZIERSURFACEND_HPP_
#define RWLIBS_NURBS_BEZIERSURFACEND_HPP_

/**
 * @file BezierSurfaceND.hpp
 *
 * \copydoc rwlibs::nurbs::BezierSurfaceND
 */

#include "BezierCurveND.hpp"
#include "ParametricUtility.hpp"

namespace rwlibs {
namespace nurbs {

template<std::size_t N>
class BezierSurfaceRationalND;

//! @addtogroup nurbs
//! @{
template<std::size_t N>
class BezierSurfaceND {
public:
	typedef std::vector<std::vector<rw::math::VectorND<N> > > PointMap;

	BezierSurfaceND(const BezierSurfaceND<N> &surface):
		_points(surface.getPoints())
	{
	}

	BezierSurfaceND(const PointMap &points = PointMap()):
		_points(points)
	{
	}

	BezierSurfaceND(const BezierSurfaceRationalND<N-1> &rational):
		_points(ParametricUtility::toHomogeneous<N-1>(rational.getPoints(),rational.getWeights()))
	{
	}

	virtual ~BezierSurfaceND() {
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

	BezierCurveND<N> getCurveU(double u) const {
		std::vector<rw::math::VectorND<N> > points;
		for (std::size_t j = 0; j < getOrder().second; j++) {
			BezierCurveND<N> curve(getPointsV(j));
			points.push_back(curve.getValue(u));
		}
		return BezierCurveND<N>(points);
	}

	BezierCurveND<N> getCurveV(double v) const {
		std::vector<rw::math::VectorND<N> > points;
		for (std::size_t i = 0; i < getOrder().first; i++) {
			BezierCurveND<N> curve(getPointsU(i));
			points.push_back(curve.getValue(v));
		}
		return BezierCurveND<N>(points);
	}

	rw::math::VectorND<N> getValue(double u, double v) const {
		return getCurveU(u).getValue(v);
	}
	/*virtual rw::math::Vector3D<> evaluateTangent(double t);
		virtual rw::math::Vector3D<> evaluateNormal(double t);
		virtual rw::math::Vector3D<> evaluateCurvature(double t);
		virtual rw::math::Vector3D<> evaluateTorsion(double t);*/

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

	BezierSurfaceND<N>* elevateU(std::size_t times = 1) const {
		PointMap points;
		for (unsigned int i = 0; i <= getDegree().first+times; i++) {
			std::vector<rw::math::VectorND<N> > empty;
			points.push_back(empty);
		}
		for (unsigned int j = 0; j <= getDegree().second; j++) {
			for (unsigned int i = 0; i <= getDegree().first+times; i++) {
				rw::math::VectorND<N> point;
				for (std::size_t l = 0; l < N; l++)
					point[i] = 0;
				unsigned int low = 0;
				unsigned int high = i;
				if (times < i)
					low = i-times;
				if (getDegree().first < i)
					high = getDegree().first;
				for (unsigned int k = low; k <= high; k++) {
					double c = ((double)ParametricUtility::binomial(getDegree().first,k)*ParametricUtility::binomial(times,i-k))/((double)ParametricUtility::binomial(getDegree().first+times,i));
					point += c*_points[k][j];
				}
				points[i].push_back(point);
			}
		}
		return new BezierSurfaceND<N>(points);
	}

	BezierSurfaceND<N>* elevateV(std::size_t times = 1) const {
		PointMap points;
		for (unsigned int i = 0; i <= getDegree().first; i++) {
			std::vector<rw::math::VectorND<N> > vec;
			for (unsigned int j = 0; j <= getDegree().second+times; j++) {
				rw::math::VectorND<N> point;
				for (std::size_t l = 0; l < N; l++)
					point[i] = 0;
				unsigned int low = 0;
				unsigned int high = i;
				if (times < j)
					low = j-times;
				if (getDegree().second < j)
					high = getDegree().second;
				for (unsigned int k = low; k <= high; k++) {
					double c = ((double)ParametricUtility::binomial(getDegree().second,k)*ParametricUtility::binomial(times,j-k))/((double)ParametricUtility::binomial(getDegree().second+times,j));
					point += c*_points[i][k];
				}
				vec.push_back(point);
			}
			points.push_back(vec);
		}
		return new BezierSurfaceND<N>(points);
	}

	BezierSurfaceND<N> getDerivativeCurveU(std::size_t times = 1) const {
		if (times < 1)
			return *this;
		if (times >= getDegree().first) {
			std::stringstream str;
			str << "The bezier surface can not be differentiated " << times << " times in the U direction, as the degree is " << getDegree().first << ". Maximum is " << getDegree().first-1 << " times.";
			RW_THROW(str.str());
		}
		PointMap points;
		double n = getDegree().first;
		for (std::size_t i = 0; i < _points.size()-1; i++) {
			std::vector<rw::math::VectorND<N> > empty;
			points.push_back(empty);
		}
		for (std::size_t i = 0; i < _points.size()-1; i++) {
			for (std::size_t j = 0; j < _points[i].size(); j++) {
				points[i].push_back(n*(_points[i+1][j]-_points[i][j]));
			}
		}
		BezierSurfaceND<N> derivative(points);
		derivative = derivative.getDerivativeCurveU(times-1);
		return derivative;
	}

	BezierSurfaceND<N> getDerivativeCurveV(std::size_t times = 1) {
		if (times < 1)
			return *this;
		if (times >= getDegree().second) {
			std::stringstream str;
			str << "The bezier surface can not be differentiated " << times << " times in the V direction, as the degree is " << getDegree().second << ". Maximum is " << getDegree().second-1 << " times.";
			RW_THROW(str.str());
		}
		PointMap points;
		double n = getDegree().second;
		for (std::size_t i = 0; i < _points.size(); i++) {
			std::vector<rw::math::VectorND<N> > row;
			for (std::size_t j = 0; j < _points[i].size()-1; j++) {
				row.push_back(n*(_points[i][j+1]-_points[i][j]));
			}
			points.push_back(row);
		}
		BezierSurfaceND<N> derivative(points);
		derivative = derivative.getDerivativeCurveV(times-1);
		return derivative;
	}

	BezierSurfaceND<N> getDerivativeCurve(std::size_t timesU = 1, std::size_t timesV = 1) {
		return getDerivativeCurveU(timesU).getDerivativeCurveV(timesV);
	}

	BezierSurfaceND<N>* elevate(std::size_t timesU = 1, std::size_t timesV = 1) const {
		BezierSurfaceND<N>* elevU = elevateU(timesU);
		BezierSurfaceND<N>* res = elevU->elevateV(timesV);
		delete elevU;
		return res;
	}

	BezierSurfaceND<N>& operator=(const BezierSurfaceND<N> &rhs) {
		_points = rhs.getPoints();
		return *this;
	}

	const BezierSurfaceND<N> operator+( const BezierSurfaceND<N>& b) const
	{
		std::pair<BezierSurfaceND<N>, BezierSurfaceND<N> > pair = equalizeDegree<N>(b);
		BezierSurfaceND<N> first = pair.first;
		BezierSurfaceND<N> second = pair.second;

		PointMap newPoints;
		for (std::size_t i = 0; i < first.getPoints().size(); i++) {
			std::vector<rw::math::VectorND<N> > row;
			for (std::size_t j = 0; j < first.getPoints()[i].size(); j++) {
				row.push_back(first.getPoints()[i][j]+second.getPoints()[i][j]);
			}
			newPoints.push_back(row);
		}
		return BezierSurfaceND<N>( newPoints );
	}

	const BezierSurfaceND<N> operator-( const BezierSurfaceND<N>& b) const
	{
		std::pair<BezierSurfaceND<N>, BezierSurfaceND<N> > pair = equalizeDegree<N>(b);
		BezierSurfaceND<N> first = pair.first;
		BezierSurfaceND<N> second = pair.second;

		PointMap newPoints;
		for (std::size_t i = 0; i < first.getPoints().size(); i++) {
			std::vector<rw::math::VectorND<N> > row;
			for (std::size_t j = 0; j < first.getPoints()[i].size(); j++) {
				row.push_back(first.getPoints()[i][j]-second.getPoints()[i][j]);
			}
			newPoints.push_back(row);
		}
		return BezierSurfaceND<N>( newPoints );
	}

	const BezierSurfaceRationalND<N> operator/( const BezierSurfaceND<1>& b) const
	{
		std::pair<BezierSurfaceND<N>, BezierSurfaceND<1> > pair = equalizeDegree<1>(b);
		BezierSurfaceND<N> first = pair.first;
		BezierSurfaceND<1> second = pair.second;

		std::vector<std::vector<rw::math::VectorND<N> > > newPoints;
		std::vector<std::vector<double> > weights;
		for (std::size_t i = 0; i < first.getPoints().size(); i++) {
			std::vector<rw::math::VectorND<N> > vecP;
			std::vector<double> vecW;
			for (std::size_t j = 0; j < first.getPoints()[i].size(); j++) {
				vecP.push_back(first.getPoints()[i][j] / second.getPoints()[i][j][0]);
				vecW.push_back(second.getPoints()[i][j][0]);
			}
			newPoints.push_back(vecP);
			weights.push_back(vecW);
		}
		return BezierSurfaceRationalND<N>(newPoints, weights);
	}

	const BezierSurfaceND<N> operator*(double scalar) const
	{
		PointMap newPoints;
		for (std::size_t i = 0; i < getOrder().first; i++) {
			std::vector<rw::math::VectorND<N> > vec;
			for (std::size_t j = 0; j < getOrder().second; j++) {
				vec.push_back(getPoints()[i][j]*scalar);
			}
			newPoints.push_back(vec);
		}
		return BezierSurfaceND<N>(newPoints);
	}

private:
	template<std::size_t N2>
	std::pair<BezierSurfaceND<N>, BezierSurfaceND<N2> > equalizeDegree(const BezierSurfaceND<N2> &b) const
	{
		std::pair<BezierSurfaceND<N>, BezierSurfaceND<N2> > pair;
		BezierSurfaceND<N> first = *this;
		BezierSurfaceND<N2> second = b;
		if (first.getDegree().first > second.getDegree().first)
			second = *second.elevateU(first.getDegree().first - second.getDegree().first);
		if (first.getDegree().second > second.getDegree().second)
			second = *second.elevateV(first.getDegree().second - second.getDegree().second);
		if (first.getDegree().first < second.getDegree().first)
			first = *first.elevateU(second.getDegree().first - first.getDegree().first);
		if (first.getDegree().second < second.getDegree().second)
			first = *first.elevateV(second.getDegree().second - first.getDegree().second);
		pair.first = first;
		pair.second = second;
		return pair;
	}

	PointMap _points;
	//BezierSurfaceND<N>* _derivative;
};

template<std::size_t N>
const BezierSurfaceND<N> operator*(double scalar, const BezierSurfaceND<N>& b)
{
	return b*scalar;
}

template<std::size_t N>
const BezierSurfaceND<N> operator*(const BezierSurfaceND<1>& a, const BezierSurfaceND<N>& b)
{
	std::size_t m = a.getDegree().first;
	std::size_t n = a.getDegree().second;
	std::size_t p = b.getDegree().first;
	std::size_t q = b.getDegree().second;
	std::vector<std::vector<rw::math::VectorND<N> > > points;
	std::vector<std::vector<rw::math::VectorND<1> > > pointsA = a.getPoints();
	std::vector<std::vector<rw::math::VectorND<N> > > pointsB = b.getPoints();
	for (std::size_t r = 0; r <= m+p; r++) {
		std::vector<rw::math::VectorND<N> > row;
		std::size_t maxStartI;
		if (r > p)
			maxStartI = r-p;
		else
			maxStartI = 0;
		for (std::size_t s = 0; s <= n+q; s++) {
			rw::math::VectorND<N> point;
			for (std::size_t i = 0; i < N; i++)
				point[i] = 0;
			std::size_t maxStartJ;
			if (s > q)
				maxStartJ = s-q;
			else
				maxStartJ = 0;
			for (std::size_t i = maxStartI; i <= std::min<std::size_t>(r,m); i++) {
				for (std::size_t j = maxStartJ; j <= std::min<std::size_t>(s,n); j++) {
					double frac1 = ParametricUtility::binomial(m,i)*ParametricUtility::binomial(p,r-i)/ParametricUtility::binomial(m+p,r);
					double frac2 = ParametricUtility::binomial(n,j)*ParametricUtility::binomial(q,s-j)/ParametricUtility::binomial(n+q,s);
					point += frac1*frac2*pointsA[i][j][0]*pointsB[r-i][s-j];
				}
			}
			row.push_back(point);
		}
		points.push_back(row);
	}
	return BezierSurfaceND<N>(points);
}

template<std::size_t N>
const BezierSurfaceND<N> operator*(const BezierSurfaceND<N>& a, const BezierSurfaceND<1>& b)
{
	return b*a;
}

template<std::size_t N>
const BezierSurfaceND<1> dot(const BezierSurfaceND<N>& a, const BezierSurfaceND<N>& b)
{
	std::size_t m = a.getDegree().first;
	std::size_t n = a.getDegree().second;
	std::size_t p = b.getDegree().first;
	std::size_t q = b.getDegree().second;
	std::vector<std::vector<rw::math::VectorND<N> > > points;
	std::vector<std::vector<rw::math::VectorND<1> > > pointsA = a.getPoints();
	std::vector<std::vector<rw::math::VectorND<N> > > pointsB = b.getPoints();
	for (std::size_t r = 0; r <= m+p; r++) {
		std::vector<rw::math::VectorND<N> > row;
		std::size_t maxStartI;
		if (r > p)
			maxStartI = r-p;
		else
			maxStartI = 0;
		for (std::size_t s = 0; s <= n+q; s++) {
			rw::math::VectorND<N> point;
			point[0] = 0;
			std::size_t maxStartJ;
			if (s > q)
				maxStartJ = s-q;
			else
				maxStartJ = 0;
			for (std::size_t i = maxStartI; i <= std::min<std::size_t>(r,m); i++) {
				for (std::size_t j = maxStartJ; j <= std::min<std::size_t>(s,n); j++) {
					double dotted = 0;
					for (unsigned int k = 0; k < N; k++)
						dotted += pointsA[i][j][k]*pointsB[r-i][s-j][k];
					double frac1 = ParametricUtility::binomial(m,i)*ParametricUtility::binomial(p,r-i)/ParametricUtility::binomial(m+p,r);
					double frac2 = ParametricUtility::binomial(n,j)*ParametricUtility::binomial(q,s-j)/ParametricUtility::binomial(n+q,s);
					point[0] += frac1*frac2*dotted;
				}
			}
			row.push_back(point);
		}
		points.push_back(row);
	}
	return BezierSurfaceND<N>(points);
}
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_BEZIERSURFACEND_HPP_ */
