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

#ifndef RWLIBS_NURBS_BEZIERCURVERATIONALND_HPP_
#define RWLIBS_NURBS_BEZIERCURVERATIONALND_HPP_

/**
 * @file BezierCurveRationalND.hpp
 *
 * \copydoc rwlibs::nurbs::BezierCurveRationalND
 */

#include "BezierCurveND.hpp"
#include "ParametricUtility.hpp"

namespace rwlibs {
namespace nurbs {

template<std::size_t N>
class BezierCurveND;

//! @addtogroup nurbs
//! @{
template<std::size_t N>
class BezierCurveRationalND {
public:
	BezierCurveRationalND(const BezierCurveRationalND<N> &curve):
		_points(curve.getPoints()),
		_weights(curve.getWeights()),
		_homogeneous(new BezierCurveND<N+1>(*curve._homogeneous)),
		_denominator(NULL),
		_numerator(NULL),
		_derivedFrom(NULL),
		_derivative(NULL)
	{
		if (curve._denominator != NULL)
			_denominator = new BezierCurveND<1>(*curve._denominator);
		if (curve._numerator != NULL)
			_numerator = new BezierCurveND<N>(*curve._numerator);
		if (curve._derivative != NULL)
			_derivative = new BezierCurveRationalND<N>(*curve._derivative);
	}

	BezierCurveRationalND(const std::vector<rw::math::VectorND<N> > &points = std::vector<rw::math::VectorND<N> >(), const std::vector<double> &weights = std::vector<double>()):
		_points(points),
		_weights(weights),
		_homogeneous(new BezierCurveND<N+1>(ParametricUtility::toHomogeneous<N>(points,weights))),
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

	BezierCurveRationalND(const std::vector<rw::math::VectorND<N+1> > &pointsHomogeneous):
		_points(ParametricUtility::fromHomogeneous<N>(pointsHomogeneous)),
		_weights(ParametricUtility::getWeights<N>(pointsHomogeneous)),
		_homogeneous(new BezierCurveND<N+1>(pointsHomogeneous)),
		_denominator(NULL),
		_numerator(NULL),
		_derivedFrom(NULL),
		_derivative(NULL)
	{
	}

	BezierCurveRationalND(const BezierCurveND<N+1> &nonrational):
		_points(ParametricUtility::fromHomogeneous<N>(nonrational.getPoints())),
		_weights(ParametricUtility::getWeights<N>(nonrational.getPoints())),
		_homogeneous(new BezierCurveND<N+1>(nonrational)),
		_denominator(NULL),
		_numerator(NULL),
		_derivedFrom(NULL),
		_derivative(NULL)
	{
	}

	BezierCurveRationalND(const BezierCurveND<N> &curve):
		_points(curve.getPoints()),
		_weights(std::vector<double>()),
		_homogeneous(new BezierCurveND<N+1>(ParametricUtility::toHomogeneous<N>(curve.getPoints()))),
		_denominator(NULL),
		_numerator(NULL),
		_derivedFrom(NULL),
		_derivative(NULL)
	{
		for (std::size_t i = 0; i < curve.getPoints().size(); i++)
			_weights.push_back(1);
	}

	~BezierCurveRationalND() {
		delete _homogeneous;
		if (_denominator != NULL)
			delete _denominator;
		if (_numerator != NULL)
			delete _numerator;
	}

	std::vector<rw::math::VectorND<N> > getPoints() const {
		return _points;
	}

	std::vector<rw::math::VectorND<N+1> > getPointsHomogeneous() const {
		return _homogeneous->getPoints();
	}

	std::vector<double> getWeights() const {
		return _weights;
	}

	rw::math::VectorND<N> getValue(double t) const {
		return ParametricUtility::fromHomogeneous<N>(_homogeneous->getValue(t));
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

	BezierCurveRationalND<N>* getDerivativeCurve(std::size_t times = 1) {
		if (_derivedFrom == NULL) {
			BezierCurveRationalND<N>* current = this;
			BezierCurveND<N> num = getNumerator();
			BezierCurveND<1> den = getDenominator();
			for (std::size_t i = 1; i <= times; i++) {
				if (current->_derivative == NULL) {
					// Use the Leibniz formula
					BezierCurveND<1> denPow = den;
					for (std::size_t j = 1; j < i; j++)
						denPow = dot(denPow,den);
					BezierCurveND<N> newNum = (*num.getDerivativeCurve(i))*denPow;
					for (std::size_t j = 1; j <= i; j++) {
						BezierCurveND<1> denPowJ = den;
						for (std::size_t k = 1; k < j-1; k++)
							denPowJ = dot(denPowJ,den);
						BezierCurveND<N> X = getDerivativeCurve(i-j)->getNumerator();
						if (j > 1)
							newNum = newNum - ParametricUtility::binomial(i,j)*(*den.getDerivativeCurve(j))*X*denPowJ;
						else
							newNum = newNum - ParametricUtility::binomial(i,j)*(*den.getDerivativeCurve(j))*X;
					}
					denPow = dot(denPow,den);
					BezierCurveRationalND<N>* curve = new BezierCurveRationalND<N>(newNum/denPow);
					current->_derivative = curve;
					current->_derivative->setDerivedFrom(current);
				}
				current = current->_derivative;
			}
			return current;
		} else {
			BezierCurveRationalND<N>* top = _derivedFrom;
			std::size_t thisDer = 1;
			while (top->getDerivedFrom() != NULL) {
				top = top->getDerivedFrom();
				thisDer++;
			}
			return top->getDerivativeCurve(times+thisDer);
		}
	}

	BezierCurveRationalND<N>* elevate(std::size_t times = 1) const {
		return new BezierCurveRationalND<N>(_homogeneous->elevate(times)->getPoints());
	}

	/*virtual BezierCurve& operator+(const BezierCurve &rhs);
	virtual BezierCurve& operator-(const BezierCurve &rhs);
	virtual BezierCurve& operator*(const BezierCurve &rhs);
	virtual BezierCurve& operator/(const BezierCurve &rhs);
	virtual BezierCurve& operator==(const BezierCurve &rhs);
	virtual BezierCurve& operator!=(const BezierCurve &rhs);*/

	BezierCurveRationalND<N>& operator=(const BezierCurveRationalND<N> &rhs) {
		_points = rhs.getPoints();
		_weights = rhs.getWeights();
		_homogeneous =  new BezierCurveND<N+1>(*rhs._homogeneous);
		if (rhs._denominator != NULL)
			_denominator = new BezierCurveND<1>(*rhs._denominator);
		if (rhs._numerator != NULL)
			_numerator = new BezierCurveND<N>(*rhs._numerator);
		if (rhs._derivative != NULL)
			_derivative = new BezierCurveRationalND<N>(*rhs._derivative);
		return *this;
	}

	const BezierCurveRationalND<N> operator+( const BezierCurveRationalND<N>& b) const
	{
		BezierCurveRationalND<N> first = *this;
		BezierCurveRationalND<N> second = b;
		BezierCurveND<1> den = dot(first.getDenominator(),second.getDenominator());
		BezierCurveRationalND<N> res = (first.getNumerator()*second.getDenominator()+second.getNumerator()*first.getDenominator())/den;
		return res;
	}

	const BezierCurveRationalND<N> operator-( const BezierCurveRationalND<N>& b) const
	{
		BezierCurveRationalND<N> first = *this;
		BezierCurveRationalND<N> second = b;
		BezierCurveND<1> den = dot(first.getDenominator(),second.getDenominator());
		BezierCurveRationalND<N> res = (first.getNumerator()*second.getDenominator()-second.getNumerator()*first.getDenominator())/den;
		return res;
	}

	const BezierCurveRationalND<N> operator/( const BezierCurveND<1>& b)
	{
		return getNumerator()/dot(getDenominator(),b);
	}

	const BezierCurveRationalND<N> operator/( BezierCurveRationalND<1>& b)
	{
		return (getNumerator()*b.getDenominator())/(getDenominator()*b.getNumerator());
	}

	const BezierCurveRationalND<N> operator*( const BezierCurveND<1>& b)
	{
		BezierCurveRationalND<N> res = (getNumerator()*b)/getDenominator();
		return res;
	}

private:
	BezierCurveND<1> getDenominator() {
		if (_denominator == NULL) {
			std::vector<rw::math::VectorND<1> > points;
			for (std::size_t i = 0; i < _weights.size(); i++) {
				rw::math::VectorND<1> w;
				w[0] = _weights[i];
				points.push_back(w);
			}
			_denominator = new BezierCurveND<1>(points);
		}
		return *_denominator;
	}

	BezierCurveND<N> getNumerator() {
		if (_numerator == NULL) {
			std::vector<rw::math::VectorND<N> > points;
			for (std::size_t i = 0; i < _points.size(); i++)
				points.push_back(_weights[i]*_points[i]);
			_numerator = new BezierCurveND<N>(points);
		}
		return *_numerator;
	}

	BezierCurveRationalND<N>* getDerivedFrom() {
		return _derivedFrom;
	}

	void setDerivedFrom(BezierCurveRationalND<N>* curve) {
		_derivedFrom = curve;
	}

	std::vector<rw::math::VectorND<N> > _points;
	std::vector<double> _weights;

	BezierCurveND<N+1>* _homogeneous;
	BezierCurveND<1>* _denominator;
	BezierCurveND<N>* _numerator;
	BezierCurveRationalND<N>* _derivedFrom;
	BezierCurveRationalND<N>* _derivative;
};

template<std::size_t N>
const BezierCurveRationalND<N> operator*(const BezierCurveND<1>& a, BezierCurveRationalND<N>& b)
{
	return b*a;
}
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_BEZIERCURVERATIONALND_HPP_ */
