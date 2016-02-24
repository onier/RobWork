/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "PolynomialSolver.hpp"

#include <limits>
#include <complex>

#include <rw/common/macros.hpp>
#include <stddef.h>

using namespace rw::math;

PolynomialSolver::PolynomialSolver(const std::vector<double> &coefficients):
	_derivative(NULL),
	_guess(0),
	_isComplex(false)
{
	for (std::size_t i = 0; i < coefficients.size(); i++)
		_coefficients.push_back(coefficients[i]);
}

PolynomialSolver::PolynomialSolver(const std::vector<std::complex<double> > &coefficients):
	_coefficients(coefficients),
	_derivative(NULL),
	_guess(0),
	_isComplex(true)
{
}

PolynomialSolver::~PolynomialSolver()
{
	if (_derivative != NULL)
		delete _derivative;
}

void PolynomialSolver::setInitialGuess(std::complex<double> guess)
{
	_guess = guess;
}

std::vector<double> PolynomialSolver::getRealSolutions()
{
	static const double EPS = 1.0e-14;

	std::vector<std::complex<double> > sol = getSolutions();
	std::vector<double> res;

	for (std::size_t i = 0; i < sol.size(); i++) {
		std::complex<double> x = sol[i];
		if (fabs(x.imag()) <= 2.0*EPS*fabs(x.real())) {
			res.push_back(x.real());
		}
	}

	return res;
}

std::vector<std::complex<double> > PolynomialSolver::getSolutions()
{
	std::vector<std::complex<double> > res;

	// Find tentative roots
	PolynomialSolver* pol = this;
	for (std::size_t i = 0; i < _coefficients.size()-1; i++) {
		std::complex<double> x = _guess;
		x = pol->laguerre(x);
		res.push_back(x);
		PolynomialSolver* old = pol;
		pol = pol->deflate(x);
		if (old != this)
			delete old;
	}
	if (pol != this)
		delete pol;

	// Polish
	for (std::size_t i = 0; i < _coefficients.size()-1; i++) {
		res[i] = laguerre(res[i]);
	}

	return res;
}

std::complex<double> PolynomialSolver::evaluate(std::complex<double> x, double &err) const {
	// Horner's Method
	static const double EPS = std::numeric_limits<double>::epsilon();
	std::complex<double> res = _coefficients.front();
	err = abs(res);
	for (std::size_t i = 1; i < _coefficients.size(); i++) {
		res = _coefficients[i]+res*x;
		err = abs(res) + abs(x)*err;
	}
	err *= EPS;
	return res;
}

std::complex<double> PolynomialSolver::evaluateDerivative(std::complex<double> x, std::size_t times) {
	double err;
	return getDerivative(times)->evaluate(x,err);
}

PolynomialSolver* PolynomialSolver::getDerivative(std::size_t times) {
	PolynomialSolver* der = this;
	if (times > 0) {
		if (_derivative == NULL) {
			std::vector<std::complex<double> > coefficients;
			std::size_t n = _coefficients.size();
			if (n == 1)
				return this;
			for (std::size_t i = 0; i < n-1; i++) {
				std::complex<double> c = ((double)(n-1-i))*_coefficients[i];
				coefficients.push_back(c);
			}
			_derivative = new PolynomialSolver(coefficients);
		}
		der = _derivative->getDerivative(times-1);
	}
	return der;
}

PolynomialSolver* PolynomialSolver::deflate(std::complex<double> x) const {
	std::vector<std::complex<double> > newCoef;
	std::complex<double> b = _coefficients.front();
	for (std::size_t i = 1; i < _coefficients.size(); i++) {
		newCoef.push_back(b);
		b = x*b+_coefficients[i];
	}
	return new PolynomialSolver(newCoef);
}

std::complex<double> PolynomialSolver::laguerre(std::complex<double> x) {
	RW_THROW("PolynomialSolver::laguerre not implemented.");
}
