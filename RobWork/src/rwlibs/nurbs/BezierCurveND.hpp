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

#ifndef RWLIBS_NURBS_BEZIERCURVEND_HPP_
#define RWLIBS_NURBS_BEZIERCURVEND_HPP_

/**
 * @file BezierCurveND.hpp
 *
 * \copydoc rwlibs::nurbs::BezierCurveND
 */

#include "ParametricUtility.hpp"

namespace rwlibs {
namespace nurbs {

template<std::size_t N>
class BezierCurveRationalND;

//! @addtogroup nurbs
//! @{
template<std::size_t N>
class BezierCurveND {
public:
	BezierCurveND(const BezierCurveND<N> &curve):
		_points(curve.getPoints()),
		_derivative(NULL)
	{
	}

	BezierCurveND(const std::vector<rw::math::VectorND<N> > &points = std::vector<rw::math::VectorND<N> >()):
		_points(points),
		_derivative(NULL)
	{
	}

	BezierCurveND(const BezierCurveRationalND<N-1> &rational):
		_points(ParametricUtility::toHomogeneous<N-1>(rational.getPoints(),rational.getWeights())),
		_derivative(NULL)
	{
	}

	virtual ~BezierCurveND() {
	}

	std::vector<rw::math::VectorND<N> > getPoints() const {
		return _points;
	}

	rw::math::VectorND<N> getValue(double t) const {
		return deCasteljau(getDegree(),0,t);
	}
	/*virtual rw::math::Vector3D<> evaluateTangent(double t);
	virtual rw::math::Vector3D<> evaluateNormal(double t);
	virtual rw::math::Vector3D<> evaluateCurvature(double t);
	virtual rw::math::Vector3D<> evaluateTorsion(double t);*/

	std::size_t getDegree() const {
		if (_points.size() < 1)
			RW_THROW("Bezier Curve is not valid (no control points)");
		return _points.size()-1;
	}

	std::size_t getOrder() const {
		if (_points.size() < 1)
			RW_THROW("Bezier Curve is not valid (no control points)");
		return _points.size();
	}

	BezierCurveND<N>* getDerivativeCurve(std::size_t times = 1) {
		if (times >= getDegree()) {
			std::stringstream str;
			str << "The bezier curve can not be differentiated " << times << " times, as the degree is " << getDegree() << ". Maximum is " << getDegree()-1 << " times.";
			RW_THROW(str.str());
		}
		if (_derivative == NULL) {
			std::vector<rw::math::VectorND<N> > points;
			double n = getDegree();
			for (std::size_t i = 0; i < _points.size()-1; i++) {
				points.push_back(n*(_points[i+1]-_points[i]));
			}
			_derivative = new BezierCurveND<N>(points);
		}
		BezierCurveND<N>* der = _derivative;
		for(std::size_t i = 1; i < times; i++)
			der = der->getDerivativeCurve();
		return der;
	}

	BezierCurveND<N>* elevate(std::size_t times = 1) const {
		std::vector<rw::math::VectorND<N> > points;
		for (unsigned int i = 0; i <= getDegree()+times; i++) {
			rw::math::VectorND<N> point;
			unsigned int low = 0;
			unsigned int high = i;
			if (times < i)
				low = i-times;
			if (getDegree() < i)
				high = getDegree();
			for (unsigned int j = low; j <= high; j++) {
				double c = ((double)ParametricUtility::binomial(getDegree(),j)*ParametricUtility::binomial(times,i-j))/((double)ParametricUtility::binomial(getDegree()+times,i));
				point += c*_points[j];
			}
			points.push_back(point);
		}
		return new BezierCurveND<N>(points);
	}

	//virtual BezierCurve& operator=(const BezierCurve &rhs);
	/*virtual BezierCurveND<N>& operator+(const BezierCurveND<N> &rhs) {

	}*/
	/*virtual BezierCurve& operator-(const BezierCurve &rhs);
	virtual BezierCurve& operator*(const BezierCurve &rhs);
	virtual BezierCurve& operator/(const BezierCurve &rhs);
	virtual BezierCurve& operator==(const BezierCurve &rhs);
	virtual BezierCurve& operator!=(const BezierCurve &rhs);*/

	BezierCurveND<N>& operator=(const BezierCurveND<N> &rhs) {
		_points = rhs.getPoints();
		/*if (rhs._derivative != NULL)
			_derivative = new BezierCurveND<N>(*rhs._derivative);*/
		_derivative = NULL;
		return *this;
	}

	const BezierCurveND<N> operator+( const BezierCurveND<N>& b) const
	{
		BezierCurveND<N> first = *this;
		BezierCurveND<N> second = b;
		if (first.getDegree() > second.getDegree())
			second = *second.elevate(first.getDegree() - second.getDegree());
		if (first.getDegree() < second.getDegree())
			first = *first.elevate(second.getDegree() - first.getDegree());
		std::vector<rw::math::VectorND<N> > newPoints;
		for (std::size_t i = 0; i < first.getPoints().size(); i++) {
			newPoints.push_back(first.getPoints()[i]+second.getPoints()[i]);
		}
		return BezierCurveND<N>( newPoints );
	}

	const BezierCurveND<N> operator-( const BezierCurveND<N>& b) const
	{
		BezierCurveND<N> first = *this;
		BezierCurveND<N> second = b;
		if (first.getDegree() > second.getDegree())
			second = *second.elevate(first.getDegree() - second.getDegree());
		if (first.getDegree() < second.getDegree())
			first = *first.elevate(second.getDegree() - first.getDegree());
		std::vector<rw::math::VectorND<N> > newPoints;
		for (std::size_t i = 0; i < first.getPoints().size(); i++) {
			newPoints.push_back(first.getPoints()[i]-second.getPoints()[i]);
		}
		return BezierCurveND<N>( newPoints );
	}

	const BezierCurveRationalND<N> operator/( const BezierCurveND<1>& b) const
	{
		BezierCurveND<N> first = *this;
		BezierCurveND<1> second = b;
		if (first.getDegree() > second.getDegree())
			second = *second.elevate(first.getDegree() - second.getDegree());
		if (first.getDegree() < second.getDegree())
			first = *first.elevate(second.getDegree() - first.getDegree());

		std::vector<rw::math::VectorND<N> > newPoints;
		std::vector<double> weights;
		for (std::size_t i = 0; i < first.getPoints().size(); i++) {
			newPoints.push_back(first.getPoints()[i] / second.getPoints()[i][0]);
			weights.push_back(second.getPoints()[i][0]);
		}
		return BezierCurveRationalND<N>(newPoints, weights);
	}

	const BezierCurveND<N> operator*(double scalar) const
	{
		std::vector<rw::math::VectorND<N> > newPoints;
		for (std::size_t i = 0; i < getPoints().size(); i++) {
			newPoints.push_back(getPoints()[i]*scalar);
		}
		BezierCurveND<N> res(newPoints);
		return res;
	}

private:
	rw::math::VectorND<N> deCasteljau(std::size_t i, std::size_t j, double t) const {
		if (i==0)
			return _points[j];
		else
			return (1.-t)*deCasteljau(i-1,j,t)+t*deCasteljau(i-1,j+1,t);
	}

	std::vector<rw::math::VectorND<N> > _points;
	BezierCurveND<N>* _derivative;
};

/*const BezierCurveND<1> operator*(const BezierCurveND<1>& a, const BezierCurveND<1>& b)
{
	// 1992: Free form surface analysis using a hybrid of symbolic and numeric computation
	std::size_t p = a.getDegree();
	std::size_t q = b.getDegree();
	std::vector<rw::math::VectorND<1> > points;
	std::vector<rw::math::VectorND<1> > pointsA = a.getPoints();
	std::vector<rw::math::VectorND<1> > pointsB = b.getPoints();
	for (std::size_t k = 0; k <= p+q; k++) {
		rw::math::VectorND<1> point;
		for (std::size_t l = std::max<std::size_t>(0,k-q); l <= std::min<std::size_t>(p,k); l++) {
			point[0] += ParametricUtility::binomial(p,l)*ParametricUtility::binomial(q,k-l)*pointsA[l][0]*pointsB[k-l][0]/ParametricUtility::binomial(p+q,k);
		}
		points.push_back(point);
	}
	BezierCurveND<1> res(points);
	return res;
}*/

template<std::size_t N>
const BezierCurveND<N> operator*(double scalar, const BezierCurveND<N>& b)
{
	return b*scalar;
}

template<std::size_t N>
const BezierCurveND<N> operator*(const BezierCurveND<1>& a, const BezierCurveND<N>& b)
{
	std::size_t p = a.getDegree();
	std::size_t q = b.getDegree();
	std::vector<rw::math::VectorND<N> > points;
	std::vector<rw::math::VectorND<1> > pointsA = a.getPoints();
	std::vector<rw::math::VectorND<N> > pointsB = b.getPoints();
	for (std::size_t k = 0; k <= p+q; k++) {
		rw::math::VectorND<N> point;
		std::size_t maxStart;
		if (k > q)
			maxStart = k-q;
		else
			maxStart = 0;
		for (std::size_t l = maxStart; l <= std::min<std::size_t>(p,k); l++) {
			point += ParametricUtility::binomial(p,l)*ParametricUtility::binomial(q,k-l)*pointsA[l][0]*pointsB[k-l]/ParametricUtility::binomial(p+q,k);
		}
		points.push_back(point);
	}
	BezierCurveND<N> res(points);
	return res;
}

template<std::size_t N>
const BezierCurveND<N> operator*(const BezierCurveND<N>& a, const BezierCurveND<1>& b)
{
	return b*a;
}

template<std::size_t N>
const BezierCurveND<1> dot(const BezierCurveND<N>& a, const BezierCurveND<N>& b)
{
	std::size_t p = a.getDegree();
	std::size_t q = b.getDegree();
	std::vector<rw::math::VectorND<1> > points;
	std::vector<rw::math::VectorND<N> > pointsA = a.getPoints();
	std::vector<rw::math::VectorND<N> > pointsB = b.getPoints();
	for (std::size_t k = 0; k <= p+q; k++) {
		rw::math::VectorND<1> point;
		point[0] = 0;
		std::size_t maxStart;
		if (k > q)
			maxStart = k-q;
		else
			maxStart = 0;
		for (std::size_t l = maxStart; l <= std::min<std::size_t>(p,k); l++) {
			double dotted = 0;
			for (unsigned int i = 0; i < N; i++)
				dotted += pointsA[l][i]*pointsB[k-l][i];
			point[0] += ParametricUtility::binomial(p,l)*ParametricUtility::binomial(q,k-l)*dotted/ParametricUtility::binomial(p+q,k);
		}
		points.push_back(point);
	}
	return BezierCurveND<1>(points);
}
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_BEZIERCURVEND_HPP_ */
