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

#include "GeometricUtil.hpp"
#include <rw/math/Constants.hpp>
#include <rw/math/Polynomial.hpp>
#include <rw/math/PolynomialSolver.hpp>

#include <limits>
#include <iomanip>

using namespace rw::common;
using namespace rw::math;
using namespace rwsim::contacts;

struct GeometricUtil::FuncCircleCircleDist {
	double a[10];
	const double a6half;
	const double two_a4_minus_a5;
	FuncCircleCircleDist(double a9, double a8, double a7, double a6, double a5, double a4, double a3, double a2, double a1, double a0):
		a6half(a6/2),
		two_a4_minus_a5((a4-a5)*2)
	{
		a[0] = a0;
		a[1] = a1;
		a[2] = a2;
		a[3] = a3;
		a[4] = a4;
		a[5] = a5;
		a[6] = a6;
		a[7] = a7;
		a[8] = a8;
		a[9] = a9;
	};
	bool singular(const double x) const {
		const double cosx = cos(x);
		const double sinx = sin(x);
		const double sqrootVal = a[5]*cosx*cosx+a[4]*sinx*sinx+a[3]*cosx+a[2]*sinx+a[1]*cosx*sinx+a[0];
		return sqrootVal <= 0;
	}
	double operator()(const double x) const {
		const double cosx = cos(x);
		const double sinx = sin(x);
		const double val = a[9]*cosx+a[8]*sinx+a[7]+a[6]*std::sqrt(a[5]*cosx*cosx+a[4]*sinx*sinx+a[3]*cosx+a[2]*sinx+a[1]*cosx*sinx+a[0]);
		return val;
	}
	double df(const double x) const {
		const double cosx = cos(x);
		const double sinx = sin(x);
		const double num = a[1]*cosx*cosx-a[1]*sinx*sinx+a[2]*cosx-a[3]*sinx+two_a4_minus_a5*cosx*sinx;
		const double den = a[5]*cosx*cosx+a[4]*sinx*sinx+a[3]*cosx+a[2]*sinx+a[1]*cosx*sinx+a[0];
		const double val = a[8]*cosx-a[9]*sinx+a6half*num/std::sqrt(den);
		return val;
	}
	double ddf(const double x) const {
		const double cosx = cos(x);
		const double sinx = sin(x);
		const double num = a[1]*cosx*cosx-a[1]*sinx*sinx+a[2]*cosx-a[3]*sinx+two_a4_minus_a5*cosx*sinx;
		const double den = a[5]*cosx*cosx+a[4]*sinx*sinx+a[3]*cosx+a[2]*sinx+a[1]*cosx*sinx+a[0];
		const double numD = two_a4_minus_a5*cosx*cosx - two_a4_minus_a5*sinx*sinx - a[3]*cosx - a[2]*sinx - 4*a[1]*cosx*sinx;
		const double denD = num;
		const double val = -a[9]*cosx-a[8]*sinx+a6half*(numD/(std::sqrt(den))+num*denD/(2*den*std::sqrt(den)));
		return val;
	}
	Polynomial<> getDfZeroPol() const {
		Polynomial<> pol(8);
		pol[0] = -8*(-2*a[1]*a[2]*a[3] + a[0]*(std::pow(a[1],2) + std::pow(a[3],2)) + a[4]*(std::pow(a[1],2) + std::pow(a[3],2)))*std::pow(a[6],2)*std::pow(a[9],2) + 16*(a[0] - a[2] + a[4])*(a[0] + a[2] + a[4])*std::pow(a[9],4) + std::pow(a[6],4)*std::pow(std::pow(a[1],2) - std::pow(a[3],2),2);
		pol[1] = 16*a[8]*a[9]*(-2*a[1]*a[3]*(a[0] + a[4]) + a[2]*std::pow(a[1],2) + a[2]*std::pow(a[3],2))*std::pow(a[6],2) - 4*(a[1] - a[3])*(a[1] + a[3])*(a[1]*a[2] + 2*a[3]*(-a[4] + a[5]))*std::pow(a[6],4) +
				   8*(2*a[0]*(a[1]*a[2] + 2*a[3]*(a[4] - a[5])) - 2*a[1]*a[2]*(a[4] - 2*a[5]) + a[3]*std::pow(a[1],2) - a[3]*(4*a[4]*(-a[4] + a[5]) + 2*std::pow(a[2],2) + std::pow(a[3],2)))*std::pow(a[6],2)*std::pow(a[9],2) + 32*(-(a[1]*a[2]) + a[3]*(a[0] + a[4]))*std::pow(a[9],4);
		pol[2] = 2*(16*a[2]*a[3]*(a[0] - a[4] + 2*a[5])*a[8]*a[9]*std::pow(a[6],2) + 8*a[8]*a[9]*std::pow(a[1],3)*std::pow(a[6],2) - 4*std::pow(a[1],4)*std::pow(a[6],4) +
			     std::pow(a[3],2)*std::pow(a[6],2)*((-std::pow(a[2],2) - std::pow(a[3],2) + 12*std::pow(a[4] - a[5],2))*std::pow(a[6],2) - 4*(a[0] + a[4])*std::pow(a[8],2)) -
			     4*((a[0]*std::pow(a[2],2) - 3*a[4]*std::pow(a[2],2) - 2*a[0]*std::pow(a[3],2) - 7*a[4]*std::pow(a[3],2) + a[5]*(-8*a[4]*(a[0] + a[4]) + 4*std::pow(a[2],2) + 5*std::pow(a[3],2)) + 4*a[0]*std::pow(a[4],2) + 4*std::pow(a[4],3) + 4*(a[0] + a[4])*std::pow(a[5],2))*
			         std::pow(a[6],2) + 4*(a[0] - a[2] + a[4])*(a[0] + a[2] + a[4])*std::pow(a[8],2))*std::pow(a[9],2) - 8*a[1]*std::pow(a[6],2)*
			      (2*a[8]*a[9]*std::pow(a[2],2) + a[8]*a[9]*(-4*(a[0] + a[4])*(a[4] - a[5]) + std::pow(a[3],2)) + a[2]*a[3]*((a[4] - a[5])*std::pow(a[6],2) - std::pow(a[8],2) + 4*std::pow(a[9],2))) +
			     std::pow(a[1],2)*((3*std::pow(a[2],2) + 5*std::pow(a[3],2) - 4*std::pow(a[4] - a[5],2))*std::pow(a[6],4) - 4*(a[0] + a[4])*std::pow(a[6],2)*std::pow(a[8],2) + 4*(5*a[0] + 2*a[4] + 3*a[5])*std::pow(a[6],2)*std::pow(a[9],2) - 8*std::pow(a[9],4)) +
			     8*(-2*(a[0] + a[4])*(a[0] + 2*a[4] - a[5]) + 3*std::pow(a[2],2) + std::pow(a[3],2))*std::pow(a[9],4));
		pol[3] = 4*(4*a[2]*a[8]*a[9]*(-4*(a[4] - a[5])*(a[0] + a[5]) + std::pow(a[2],2))*std::pow(a[6],2) + 6*a[2]*std::pow(a[1],3)*std::pow(a[6],4) + 2*a[3]*(a[4] - a[5])*(std::pow(a[2],2) + 2*(std::pow(a[3],2) - 2*std::pow(a[4] - a[5],2)))*std::pow(a[6],4) -
			     2*a[3]*(-4*(a[0] + a[4])*(a[4] - a[5]) + 2*std::pow(a[2],2) + std::pow(a[3],2))*std::pow(a[6],2)*std::pow(a[8],2) + 2*std::pow(a[1],2)*std::pow(a[6],2)*(-14*a[2]*a[8]*a[9] + a[3]*(5*(-a[4] + a[5])*std::pow(a[6],2) + std::pow(a[8],2) - 3*std::pow(a[9],2))) +
			     2*a[3]*((3*std::pow(a[2],2) + 2*(-4*(a[4] - a[5])*(a[0] + 2*a[4] - a[5]) + std::pow(a[3],2)))*std::pow(a[6],2) - 8*(a[0] + a[4])*std::pow(a[8],2))*std::pow(a[9],2) + 8*a[3]*(-2*a[0] - 3*a[4] + a[5])*std::pow(a[9],4) +
			     a[1]*(8*a[3]*(3*a[0] + 4*a[4] - a[5])*a[8]*a[9]*std::pow(a[6],2) - a[2]*(std::pow(a[2],2) + 3*std::pow(a[3],2) - 4*std::pow(a[4] - a[5],2))*std::pow(a[6],4) + 4*a[2]*(a[0] - a[4] + 2*a[5])*std::pow(a[6],2)*std::pow(a[8],2) +
			        4*a[2]*(-3*(a[0] - 2*a[4] + 3*a[5])*std::pow(a[6],2) + 4*std::pow(a[8],2))*std::pow(a[9],2) + 24*a[2]*std::pow(a[9],4)));
		pol[4] = -32*a[2]*a[3]*(a[0] + a[5])*a[8]*a[9]*std::pow(a[6],2) - 80*a[8]*a[9]*std::pow(a[1],3)*std::pow(a[6],2) + 24*std::pow(a[1],4)*std::pow(a[6],4) + std::pow(a[2],4)*std::pow(a[6],4) + 96*a[4]*a[5]*std::pow(a[3],2)*std::pow(a[6],4) + std::pow(a[3],4)*std::pow(a[6],4) -
				   48*std::pow(a[3],2)*std::pow(a[4],2)*std::pow(a[6],4) - 64*a[5]*std::pow(a[4],3)*std::pow(a[6],4) + 16*std::pow(a[4],4)*std::pow(a[6],4) - 48*std::pow(a[3],2)*std::pow(a[5],2)*std::pow(a[6],4) + 96*std::pow(a[4],2)*std::pow(a[5],2)*std::pow(a[6],4) -
				   64*a[4]*std::pow(a[5],3)*std::pow(a[6],4) + 16*std::pow(a[5],4)*std::pow(a[6],4) + 64*a[0]*a[4]*a[5]*std::pow(a[6],2)*std::pow(a[8],2) + 8*a[0]*std::pow(a[3],2)*std::pow(a[6],2)*std::pow(a[8],2) + 48*a[4]*std::pow(a[3],2)*std::pow(a[6],2)*std::pow(a[8],2) -
				   40*a[5]*std::pow(a[3],2)*std::pow(a[6],2)*std::pow(a[8],2) - 32*a[0]*std::pow(a[4],2)*std::pow(a[6],2)*std::pow(a[8],2) + 64*a[5]*std::pow(a[4],2)*std::pow(a[6],2)*std::pow(a[8],2) - 32*std::pow(a[4],3)*std::pow(a[6],2)*std::pow(a[8],2) -
				   32*a[0]*std::pow(a[5],2)*std::pow(a[6],2)*std::pow(a[8],2) - 32*a[4]*std::pow(a[5],2)*std::pow(a[6],2)*std::pow(a[8],2) + 32*a[0]*a[4]*std::pow(a[8],4) + 16*std::pow(a[0],2)*std::pow(a[8],4) + 16*std::pow(a[4],2)*std::pow(a[8],4) +
				   8*(((-11*a[4] + 10*a[5])*std::pow(a[3],2) - a[0]*(std::pow(a[3],2) - 8*std::pow(a[4] - a[5],2)) + 4*(3*a[4] - a[5])*std::pow(a[4] - a[5],2))*std::pow(a[6],2) + 4*((a[0] + a[4])*(a[0] + 3*a[4] - 2*a[5]) - std::pow(a[3],2))*std::pow(a[8],2))*std::pow(a[9],2) +
				   16*a[1]*std::pow(a[6],2)*(7*a[8]*a[9]*std::pow(a[2],2) + 4*a[8]*a[9]*(-3*(a[0] + a[4])*(a[4] - a[5]) + std::pow(a[3],2)) + a[2]*a[3]*(3*(a[4] - a[5])*std::pow(a[6],2) - 3*std::pow(a[8],2) + 4*std::pow(a[9],2))) +
				   16*(6*a[0]*a[4] - 4*a[0]*a[5] - 6*a[4]*a[5] + std::pow(a[0],2) - 2*std::pow(a[3],2) + 6*std::pow(a[4],2) + std::pow(a[5],2))*std::pow(a[9],4) +
				   8*std::pow(a[1],2)*((-3*std::pow(a[2],2) - 2*std::pow(a[3],2) + 5*std::pow(a[4] - a[5],2))*std::pow(a[6],4) + (4*a[0] + a[4] + 3*a[5])*std::pow(a[6],2)*std::pow(a[8],2) + ((-8*a[0] + 3*a[4] - 11*a[5])*std::pow(a[6],2) + 4*std::pow(a[8],2))*std::pow(a[9],2) +
				      6*std::pow(a[9],4)) - 2*std::pow(a[2],2)*(-(std::pow(a[3],2)*std::pow(a[6],4)) + 4*(-2*a[4]*a[5]*std::pow(a[6],4) + std::pow(a[4],2)*std::pow(a[6],4) + std::pow(a[5],2)*std::pow(a[6],4) + a[0]*std::pow(a[6],2)*std::pow(a[8],2) - 3*a[4]*std::pow(a[6],2)*std::pow(a[8],2) +
				         4*a[5]*std::pow(a[6],2)*std::pow(a[8],2) + 2*std::pow(a[8],4) + ((-a[0] + 6*a[4] - 7*a[5])*std::pow(a[6],2) + 8*std::pow(a[8],2))*std::pow(a[9],2) + 6*std::pow(a[9],4)));
		pol[5] = -8*(2*a[2]*a[8]*a[9]*(-4*(a[4] - a[5])*(a[0] + a[5]) + std::pow(a[2],2) + std::pow(a[3],2))*std::pow(a[6],2) + 6*a[2]*std::pow(a[1],3)*std::pow(a[6],4) +
			     a[3]*((a[4] - a[5])*std::pow(a[6],2) - std::pow(a[8],2))*((std::pow(a[2],2) + std::pow(a[3],2) - 8*std::pow(a[4] - a[5],2))*std::pow(a[6],2) + 4*(a[0] + a[4])*std::pow(a[8],2)) +
			     a[3]*((-4*(a[0] + 5*a[4] - 4*a[5])*(a[4] - a[5]) + std::pow(a[2],2) + std::pow(a[3],2))*std::pow(a[6],2) - 8*(a[0] + 2*a[4] - a[5])*std::pow(a[8],2))*std::pow(a[9],2) -
			     2*std::pow(a[1],2)*std::pow(a[6],2)*(14*a[2]*a[8]*a[9] + a[3]*(4*(a[4] - a[5])*std::pow(a[6],2) - std::pow(a[8],2) + std::pow(a[9],2))) - 4*a[3]*(a[0] + 3*a[4] - 2*a[5])*std::pow(a[9],4) +
			     a[1]*(4*a[3]*(2*a[0] + 7*a[4] - 5*a[5])*a[8]*a[9]*std::pow(a[6],2) + a[2]*((-std::pow(a[2],2) - std::pow(a[3],2) + 6*std::pow(a[4] - a[5],2))*std::pow(a[6],4) + 2*(2*a[0] - 5*a[4] + 7*a[5])*std::pow(a[6],2)*std::pow(a[8],2) + 4*std::pow(a[8],4)) +
			        2*a[2]*((-2*a[0] + 9*a[4] - 11*a[5])*std::pow(a[6],2) + 8*std::pow(a[8],2))*std::pow(a[9],2) + 12*a[2]*std::pow(a[9],4)));
		pol[6] = -8*(4*a[2]*a[3]*(-a[4] + a[5])*a[8]*a[9]*std::pow(a[6],2) - 16*a[8]*a[9]*std::pow(a[1],3)*std::pow(a[6],2) + 4*std::pow(a[1],4)*std::pow(a[6],4) + 6*a[4]*a[5]*std::pow(a[3],2)*std::pow(a[6],4) - 3*std::pow(a[3],2)*std::pow(a[4],2)*std::pow(a[6],4) -
			     16*a[5]*std::pow(a[4],3)*std::pow(a[6],4) + 4*std::pow(a[4],4)*std::pow(a[6],4) - 3*std::pow(a[3],2)*std::pow(a[5],2)*std::pow(a[6],4) + 24*std::pow(a[4],2)*std::pow(a[5],2)*std::pow(a[6],4) - 16*a[4]*std::pow(a[5],3)*std::pow(a[6],4) +
			     4*std::pow(a[5],4)*std::pow(a[6],4) + 8*a[0]*a[4]*a[5]*std::pow(a[6],2)*std::pow(a[8],2) + 5*a[4]*std::pow(a[3],2)*std::pow(a[6],2)*std::pow(a[8],2) - 5*a[5]*std::pow(a[3],2)*std::pow(a[6],2)*std::pow(a[8],2) - 4*a[0]*std::pow(a[4],2)*std::pow(a[6],2)*std::pow(a[8],2) +
			     20*a[5]*std::pow(a[4],2)*std::pow(a[6],2)*std::pow(a[8],2) - 8*std::pow(a[4],3)*std::pow(a[6],2)*std::pow(a[8],2) - 4*a[0]*std::pow(a[5],2)*std::pow(a[6],2)*std::pow(a[8],2) - 16*a[4]*std::pow(a[5],2)*std::pow(a[6],2)*std::pow(a[8],2) +
			     4*std::pow(a[5],3)*std::pow(a[6],2)*std::pow(a[8],2) + 4*a[0]*a[4]*std::pow(a[8],4) - 4*a[0]*a[5]*std::pow(a[8],4) - 4*a[4]*a[5]*std::pow(a[8],4) - 2*std::pow(a[3],2)*std::pow(a[8],4) + 4*std::pow(a[4],2)*std::pow(a[8],4) +
			     ((a[4] - a[5])*(4*(a[0] + 3*a[4] - 2*a[5])*(a[4] - a[5]) - 5*std::pow(a[3],2))*std::pow(a[6],2) + 4*((a[4] - a[5])*(2*a[0] + 3*a[4] - a[5]) - std::pow(a[3],2))*std::pow(a[8],2))*std::pow(a[9],2) +
			     2*a[1]*std::pow(a[6],2)*(5*a[8]*a[9]*std::pow(a[2],2) + a[8]*a[9]*(-4*(a[4] - a[5])*(2*a[0] + 3*a[4] - a[5]) + 3*std::pow(a[3],2)) + a[2]*a[3]*(2*(a[4] - a[5])*std::pow(a[6],2) - std::pow(a[8],2) + std::pow(a[9],2))) -
			     2*(-2*(a[4] - a[5])*(a[0] + 2*a[4] - a[5]) + std::pow(a[3],2))*std::pow(a[9],4) + std::pow(a[1],2)*(-3*std::pow(a[2],2)*std::pow(a[6],4) - std::pow(a[3],2)*std::pow(a[6],4) +
			        2*(-8*a[4]*a[5]*std::pow(a[6],4) + 4*std::pow(a[4],2)*std::pow(a[6],4) + 4*std::pow(a[5],2)*std::pow(a[6],4) + 2*a[0]*std::pow(a[6],2)*std::pow(a[8],2) - 2*a[4]*std::pow(a[6],2)*std::pow(a[8],2) + 4*a[5]*std::pow(a[6],2)*std::pow(a[8],2) + std::pow(a[8],4) -
			           2*((a[0] - 2*a[4] + 3*a[5])*std::pow(a[6],2) - 2*std::pow(a[8],2))*std::pow(a[9],2) + 3*std::pow(a[9],4))) -
			     std::pow(a[2],2)*(3*a[5]*(a[8] - a[9])*(a[8] + a[9])*std::pow(a[6],2) + std::pow(a[4],2)*std::pow(a[6],4) + std::pow(a[5],2)*std::pow(a[6],4) + a[4]*std::pow(a[6],2)*(-2*a[5]*std::pow(a[6],2) - 3*std::pow(a[8],2) + 3*std::pow(a[9],2)) +
			        2*std::pow(std::pow(a[8],2) + std::pow(a[9],2),2)));
		pol[7] = 32*(a[1]*a[2] + a[3]*(-a[4] + a[5]))*(-4*a[1]*a[8]*a[9]*std::pow(a[6],2) + std::pow(a[1],2)*std::pow(a[6],4) + 2*((a[4] - a[5])*std::pow(a[6],2) + std::pow(a[8],2))*std::pow(a[9],2) + std::pow(a[9],4) + std::pow((-a[4] + a[5])*std::pow(a[6],2) + std::pow(a[8],2),2));
		pol[8] = 16*(std::pow(a[1],2) + std::pow(a[4] - a[5],2))*(-4*a[1]*a[8]*a[9]*std::pow(a[6],2) + std::pow(a[1],2)*std::pow(a[6],4) + 2*((a[4] - a[5])*std::pow(a[6],2) + std::pow(a[8],2))*std::pow(a[9],2) + std::pow(a[9],4) + std::pow((-a[4] + a[5])*std::pow(a[6],2) + std::pow(a[8],2),2));
		return pol;
	}
};

struct GeometricUtil::FuncdTrig {
	const double _A, _B, _C, _D, _E, _F;
	const double _2BminusA, _2C;
	FuncdTrig(double A, double B, double C, double D, double E, double F): _A(A), _B(B), _C(C), _D(D), _E(E), _F(F), _2BminusA(2.*(B-A)), _2C(2.*C) {};
	double operator()(const double x) const {
		const double cosx = cos(x);
		const double sinx = sin(x);
		// 8 multiplications and 4 additions
		double val = _A*cosx*cosx+_B*sinx*sinx+_C*cosx*sinx+_D*cosx+_E*sinx+_F;
		// 5 multiplications and 5 additions
		//double val = cosx*(_A*cosx+_C*sinx+_D)+sinx*(_B*sinx+_E)+_F;
		// 4 multiplications and 5 additions
		//double val = cosx*((_A-_B)*cosx+_C*sinx+_D)+_E*sinx+_F+_B;
		return val;
	}
	double df(const double x) const {
		// The df function comes very close to zero at a minimum
		// The more accurate it can be determined near zero, the better minimization works
		const double cosx = cos(x);
		const double sinx = sin(x);
		// 8 multiplications and 4 additions
		//double old = _C*cosx*cosx-_C*sinx*sinx+_2BminusA*cosx*sinx+_E*cosx-_D*sinx;
		// 5 multiplications and 4 additions
		//double val = cosx*(_C*cosx+_2BminusA*sinx+_E)-sinx*(_C*sinx+_D);
		// 6 multiplications and 4 additions
		//double val = 2*_C*cosx*cosx+_2BminusA*cosx*sinx+_E*cosx-_D*sinx-_C;
		// 4 multiplications and 4 additions
		double val = cosx*(_2C*cosx+_2BminusA*sinx+_E)-_D*sinx-_C;
		//double val2 = sinx*(-_2C*sinx+_2BminusA*cosx-_D)+_E*cosx+_C;
		return val;
	}
};

struct GeometricUtil::FuncdPol {
	rw::math::Polynomial<> _pol;
	FuncdPol(rw::math::Polynomial<> &pol): _pol(pol) {};
	double operator()(const double x) {
		//std::cout << "eval x=" << x;
		double val = _pol.evaluate(x);
		//std::cout << " val=" << val << std::endl;
		return val;
	}
	double df(const double x) {
		//std::cout << "deval x=" << x;
		double val = _pol.evaluateDerivatives(x,1)[1];
		//std::cout << " val=" << val << std::endl;
		return val;
	}
};

/*
 * Implementation of a Bracket method for minimization.
 * See Numerical Recipies 3rd edition (section 9.3). Implementation exists in mins.h.
 */

struct GeometricUtil::Bracketmethod {
	double ax,bx,cx,fa,fb,fc;
	template <class T>
	void bracket(const double a, const double b, T &func)
	{
		RW_THROW("GeometricUtil::Bracketmethod not implemented!");
	}
};

/*
 * Implementation of Brent's Method for minimization with first derivatives.
 * See Numerical Recipies 3rd edition (section 9.3). Implementation exists in mins.h.
 */
struct GeometricUtil::Dbrent : Bracketmethod {
	double xmin,fmin;
	const double tol;
	Dbrent(const double toll=3.0e-8) : xmin(0), fmin(0), tol(toll) {}
	template <class T>
	double minimize(T &funcd) {
		RW_THROW("GeometricUtil::Dbrent not implemented!");
	}
};

Vector3D<> GeometricUtil::getPerpendicular(const Vector3D<> &normal) {
	if (fabs(normal[0]) < fabs(normal[1])) {
		if (fabs(normal[0]) < fabs(normal[2]))
			return normalize(cross(Vector3D<>::x(),normal));
		else
			return normalize(cross(Vector3D<>::z(),normal));
	} else {
		if (fabs(normal[1]) < fabs(normal[2]))
			return normalize(cross(Vector3D<>::y(),normal));
		else
			return normalize(cross(Vector3D<>::z(),normal));
	}
}

Vector3D<> GeometricUtil::projectPointOnPlane(const Vector3D<> &point, const Vector3D<> &planePoint, const Vector3D<> &planeNormal) {
	return point-dot(point-planePoint,planeNormal)*planeNormal;
}

GeometricUtil::PointPair GeometricUtil::projectLineOnPlane(const Vector3D<> &point, const Vector3D<> &dir, const Vector3D<> &planePoint, const Vector3D<> &planeNormal) {
	PointPair res;
	res.first = projectPointOnPlane(point,planePoint,planeNormal);
	res.second = normalize(cross(cross(planeNormal,dir),planeNormal));
	return res;
}

Vector3D<> GeometricUtil::projectPointOnLine(const Vector3D<> &point, const Vector3D<> &linePoint, const Vector3D<> &lineDir) {
	return dot(point-linePoint,lineDir)*lineDir+linePoint;
}

Vector3D<> GeometricUtil::closestPointOnCircleToPlane(const Vector3D<> &center, double radius, const Vector3D<> &circleNormal, const Vector3D<> &planeNormal) {
	return normalize(cross(cross(planeNormal,circleNormal),circleNormal))*radius+center;
}

double GeometricUtil::pointLineDistance(const Vector3D<> &point, const Vector3D<> &linePoint, const Vector3D<> &lineDir) {
	Vector3D<> tmp = linePoint-point;
	return (tmp-dot(tmp,lineDir)*lineDir).norm2();
}

Vector3D<> GeometricUtil::linePlaneIntersection(const Vector3D<> &planePoint, const Vector3D<> &planeNormal, const Vector3D<> &linePoint, const Vector3D<> &lineDir) {
	return linePoint+dot(planeNormal,planePoint-linePoint)/dot(planeNormal,lineDir)*lineDir;
}

std::pair<Vector3D<>, Vector3D<> > GeometricUtil::planePlaneIntersection(const Vector3D<> &planeAPoint, const Vector3D<> &planeANormal, const Vector3D<> &planeBPoint, const Vector3D<> &planeBNormal) {
	std::pair<Vector3D<>, Vector3D<> > res;
	Vector3D<> point, dir;

	dir = normalize(cross(planeANormal,planeBNormal));
	point = linePlaneIntersection(planeBPoint,planeBNormal,planeAPoint,normalize(cross(planeANormal,dir)));

	res.first = point;
	res.second = dir;
	return res;
}

Vector3D<> GeometricUtil::closestPointOnCircleToPoint(const Vector3D<> &center, double radius, const Vector3D<> &circleNormal, const Vector3D<> point) {
	const Vector3D<> proj = projectPointOnPlane(point, center, circleNormal);
	return center+normalize(proj-center)*radius;
}

std::vector<GeometricUtil::PointPair> GeometricUtil::closestPointsOnCircleToLine(const Vector3D<> &center, double radius, const Vector3D<> &circleNormal, const Vector3D<> linePoint, const Vector3D<> &lineDir) {
	std::vector<PointPair> res;

	const Vector3D<> u = getPerpendicular(circleNormal);
	const Vector3D<> v = normalize(cross(circleNormal,u));

	const double r2 = radius*radius;
	const double ul = dot(u,lineDir);
	const double vl = dot(v,lineDir);
	const double cl = dot(center-linePoint,lineDir);

	const double A = -r2*ul*ul;
	const double B = -r2*vl*vl;
	const double C = -2.*r2*ul*vl;
	const double D = 2.*radius*dot(u,center-linePoint)-2.*radius*cl*ul;
	const double E = 2.*radius*dot(v,center-linePoint)-2.*radius*cl*vl;
	const double F = r2+dot(center-linePoint,center-linePoint)-cl*cl;

	//const static int prec = 19;
	//std::cout << "Ak=" << std::fixed << std::setprecision(prec) << A << "`" << prec << ";" << std::endl;
	//std::cout << "Bk=" << std::fixed << std::setprecision(prec) << B << "`" << prec << ";" << std::endl;
	//std::cout << "Ck=" << std::fixed << std::setprecision(prec) << C << "`" << prec << ";" << std::endl;
	//std::cout << "Dk=" << std::fixed << std::setprecision(prec) << D << "`" << prec << ";" << std::endl;
	//std::cout << "Ek=" << std::fixed << std::setprecision(prec) << E << "`" << prec << ";" << std::endl;
	//std::cout << "Fk=" << std::fixed << std::setprecision(prec) << F << "`" << prec << ";" << std::endl;
	//std::cout << "2*(B-A)=" << std::fixed << std::setprecision(prec) << 2*(B-A) << "`" << prec << ";" << std::endl;

	const FuncdTrig func(A, B, C, D, E, F);
	const FuncdTrig funcD(C, -C, 2*(B-A), E, -D, 0);

	//const static double tolerance = std::numeric_limits<double>::epsilon();
	//const static double tolerance = 3.0e-8;
	//const static double tolerance = 5.25e-17;
	const static double tolerance = std::numeric_limits<double>::epsilon();
	//const static double tolerance = 0;

	double solMin;
	{
		// Find brackets
		double xa = -2.*Pi/3.;
		double xb = 0;
		double xc = 2.*Pi/3.;
		const double Fa = func(xa);
		const double Fb = func(xb);
		const double Fc = func(xc);
		if (Fa < Fb && Fa < Fc) {
			xa = -4.*Pi/3.;
			xb = -2.*Pi/3.;
			xc = 0;
		} else if (Fc < Fa && Fc < Fb) {
			xa = 0;
			xb = 2.*Pi/3.;
			xc = 4.*Pi/3.;
		}

		// Setup minimizer
		Dbrent dbrent(tolerance);
		dbrent.ax = xa;
		dbrent.bx = xb;
		dbrent.cx = xc;
		dbrent.fa = Fa;
		dbrent.fb = Fb;
		dbrent.fc = Fc;
		//dbrent.bracket(xa,xc,func);

		// Find minimum
		try {
			solMin = dbrent.minimize(func);
		} catch(Exception& e) {
			RW_THROW("min failed9");
		}

		RW_ASSERT(solMin > xa && solMin < xc);
	}

	double solMax;
	{
		// Setup "opposite" function
		FuncdTrig funcNeg(-A, -B, -C, -D, -E, -F);

		// Find brackets
		double xa = -2.*Pi/3.;
		double xb = 0;
		double xc = 2.*Pi/3.;
		const double Fa = funcNeg(xa);
		const double Fb = funcNeg(xb);
		const double Fc = funcNeg(xc);
		if (Fa < Fb && Fa < Fc) {
			xa = -4.*Pi/3.;
			xb = -2.*Pi/3.;
			xc = 0;
		} else if (Fc < Fa && Fc < Fb) {
			xa = 0;
			xb = 2.*Pi/3.;
			xc = 4.*Pi/3.;
		}

		// Setup minimizer
		Dbrent dbrent(tolerance);
		dbrent.ax = xa;
		dbrent.bx = xb;
		dbrent.cx = xc;
		dbrent.fa = Fa;
		dbrent.fb = Fb;
		dbrent.fc = Fc;
		//dbrent.bracket(xa,xc,funcNeg);

		// Find maximum
		try {
			solMax = dbrent.minimize(funcNeg);
		} catch(Exception& e) {
			RW_THROW("min failed10");
		}
	}

	// Find polynomial for derivative of distance function
	Polynomial<> P(4);
	{
		const double Ad = C;
		const double Bd = -C;
		const double Cd = 2*(B-A);
		const double Dd = E;
		const double Ed = -D;

		Polynomial<> Pfourth(4);
		Pfourth[4] = (Ad-Bd)*(Ad-Bd)+Cd*Cd;
		Pfourth[3] = 2*((Ad-Bd)*Dd+Cd*Ed);
		Pfourth[2] = Dd*Dd+2*(Ad-Bd)*Bd-Cd*Cd+Ed*Ed;
		Pfourth[1] = 2*Bd*Dd-2*Cd*Ed;
		Pfourth[0] = Bd*Bd-Ed*Ed;

		// Deflate the polynomial as two solutions are already known
		Pfourth = Pfourth.deflate(cos(solMin));
		Pfourth = Pfourth.deflate(cos(solMax));
		P = Pfourth;
	}
	RW_ASSERT(P.order() == 2);

	std::vector<double> sol;
	if (4*P[2]*P[0] < P[1]*P[1]) {
		// Two more extremums was found
		const double sqt = sqrt(P[1]*P[1]-4*P[2]*P[0]);
		const double t1 = (-P[1]+sqt)/(2*P[2]);
		const double t2 = (-P[1]-sqt)/(2*P[2]);

		if (fabs(t1) <= 1+2*std::numeric_limits<double>::epsilon()) {
			if (fabs(t1) <= 1) {
				const double sol1A = +acos(t1);
				const double sol1B = -acos(t1);
				if (fabs(funcD(sol1A)) < fabs(funcD(sol1B)))
					sol.push_back(sol1A);
				else
					sol.push_back(sol1B);
			} else if (t1 < -1) {
				sol.push_back(Pi);
			} else if (t1 > 1) {
				sol.push_back(0);
			}
			RW_ASSERT(sol.size() != 0);
			RW_ASSERT(!std::isnan(sol.back()));
		}

		if (fabs(t2) <= 1+2*std::numeric_limits<double>::epsilon()) {
			if (fabs(t2) <= 1) {
				const double sol2A = +acos(t2);
				const double sol2B = -acos(t2);
				if (fabs(funcD(sol2A)) < fabs(funcD(sol2B)))
					sol.push_back(sol2A);
				else
					sol.push_back(sol2B);
			} else if (t2 < -1) {
				sol.push_back(Pi);
			} else if (t2 > 1) {
				sol.push_back(0);
			}
			RW_ASSERT(sol.size() != 0);
			RW_ASSERT(!std::isnan(sol.back()));
		}
	} else if (4*P[2]*P[0] == P[1]*P[1]) {
		// There was one additional extremum
		const double t = -P[1]/(2*P[2]);
		if (fabs(t) <= 1+std::numeric_limits<double>::epsilon()) {
			if (fabs(t) < 1) {
				const double solA = +acos(t);
				const double solB = -acos(t);

				if (funcD(solA) < funcD(solB))
					sol.push_back(solA);
				else
					sol.push_back(solB);
			} else if (t < -1) {
				sol.push_back(Pi);
			} else if (t > 1) {
				sol.push_back(0);
			}
		}
		RW_ASSERT(!std::isnan(sol.back()));
	}

	for(std::vector<double>::iterator it = sol.begin(); it != sol.end(); it++) {
		const double cur = *it;
		if (funcD.df(cur) < 0) {
			// solution is a maximum - remove it
			it = sol.erase(it);
			it--;
		}
	}
	if (sol.size() == 2) {
		// We remove the one that is closest to min
		if (fabs(sol[0]-solMin) < fabs(sol[1]-solMin))
			sol.erase(sol.begin());
		else
			sol.erase(sol.begin()+1);
	}
	RW_ASSERT(sol.size() <= 1);

	if (sol.size() == 0) {
		res.resize(1);
		//std::cout << "theta1=" << std::fixed << std::setprecision(prec) << solMin << "`" << prec << ";" << std::endl;
		res[0].first = radius*u*cos(solMin)+radius*v*sin(solMin)+center;
		res[0].second = linePoint+dot(res[0].first-linePoint,lineDir) * lineDir;
	} else {
		res.resize(2);
		double solA;
		double solB;
		if (func(solMin) > func(sol[0])) {
			solA = sol[0];
			solB = solMin;
		} else {
			solA = solMin;
			solB = sol[0];
		}
		//std::cout << "theta1=" << std::fixed << std::setprecision(prec) << solA << "`" << prec << ";" << std::endl;
		//std::cout << "theta2=" << std::fixed << std::setprecision(prec) << solB << "`" << prec << ";" << std::endl;
		res[0].first = radius*u*cos(solA)+radius*v*sin(solA)+center;
		res[0].second = linePoint+dot(res[0].first-linePoint,lineDir) * lineDir;
		res[1].first = radius*u*cos(solB)+radius*v*sin(solB)+center;
		res[1].second = linePoint+dot(res[1].first-linePoint,lineDir) * lineDir;
	}
	return res;
}

std::vector<Vector3D<> > GeometricUtil::closestPointsOnEllipseToCircle(const Vector3D<> &centerCircle, const Vector3D<> &normal, const Vector3D<> &centerEllipse, const Vector3D<> &dir1, double r1, const Vector3D<> &dir2, double r2) {
	std::vector<Vector3D<> > res;

	double c = dot(r2*dir2,r2*dir2)-dot(r1*dir1,r1*dir1);
	double d = dot(centerEllipse-centerCircle,r2*dir2);
	double e = -dot(centerEllipse-centerCircle,r1*dir1);

	double A = c*c;
	double B = 2.*c*e;
	double C = d*d+e*e-A;
	double D = -B;
	double E = -e*e;

	std::vector<double> coef;
	coef.push_back(A);
	coef.push_back(B);
	coef.push_back(C);
	coef.push_back(D);
	coef.push_back(E);
	PolynomialSolver solver(coef);
	std::vector<double> sol = solver.getRealSolutions();

	for (std::vector<double>::iterator it = sol.begin(); it < sol.end(); it++) {
		if (*it >= -1. && *it <= 1.) {
			double theta1 = +acos(*it);
			double theta2 = -acos(*it);
			double d1 = c*cos(theta1)*sin(theta1)+d*cos(theta1)+e*sin(theta1);
			double d2 = c*cos(theta2)*sin(theta2)+d*cos(theta2)+e*sin(theta2);
			double dd1 = 2*c*cos(theta1)*cos(theta1)+e*cos(theta1)-d*sin(theta1)-c;
			double dd2 = 2*c*cos(theta2)*cos(theta2)+e*cos(theta2)-d*sin(theta2)-c;
			if (fabs(d1) < fabs(d2)) {
				if (dd1 <= 0) {
					Vector3D<> p1 = r1*dir1*cos(theta1)+r2*dir2*sin(theta1)+centerEllipse;
					res.push_back(p1);
				}
			}
			if (fabs(d2) < fabs(d1)) {
				if (dd2 <= 0) {
					Vector3D<> p2 = r1*dir1*cos(theta2)+r2*dir2*sin(theta2)+centerEllipse;
					res.push_back(p2);
				}
			}
		}
	}

	return res;
}

std::pair<Vector3D<>, Vector3D<> > GeometricUtil::closestPointOnCircleToCircle(const Vector3D<> &cCircle1, const Vector3D<> &n1, double r1, const Vector3D<> &cCircle2, const Vector3D<> &n2, double r2) {
	std::pair<Vector3D<>, Vector3D<> > res;

	const Vector3D<> u = getPerpendicular(n2);
	const Vector3D<> v = normalize(cross(n2,u));

	const double c1u = dot(cCircle1,u);
	const double c1v = dot(cCircle1,v);
	const double c2u = dot(cCircle2,u);
	const double c2v = dot(cCircle2,v);
	const double n1u = dot(n1,u);
	const double n1v = dot(n1,v);
	const double n1c1 = dot(n1,cCircle1);
	const double n1c2 = dot(n1,cCircle2);
	const double c1c1 = dot(cCircle1,cCircle1);
	const double c2c2 = dot(cCircle2,cCircle2);
	const double c1c2 = dot(cCircle1,cCircle2);

	const double a9 = 2*r2*(c2u-c1u);
	const double a8 = 2*r2*(c2v-c1v);
	const double a7 = r1*r1+r2*r2+c1c1+c2c2-2*c1c2;
	const double a6 = -2*r1;
	const double a5 = -(r2*n1u)*(r2*n1u);
	const double a4 = -(r2*n1v)*(r2*n1v);
	const double a3 = 2*r2*(c2u-c1u-(n1c2-n1c1)*n1u);
	const double a2 = 2*r2*(c2v-c1v-(n1c2-n1c1)*n1v);
	const double a1 = -2*r2*r2*n1u*n1v;
	const double a0 = r2*r2+c1c1+c2c2-2*c1c2-(n1c2-n1c1)*(n1c2-n1c1);

	const double A = a9*a9-a5*a6*a6;
	const double B = a8*a8-a4*a6*a6;
	const double C = 2*a8*a9-a1*a6*a6;
	const double D = 2*a7*a9-a3*a6*a6;
	const double E = 2*a7*a8-a2*a6*a6;
	const double F = a0*a6*a6;

	// Setup function
	FuncdTrig func(A, B, C, D, E, F);

	// Find brackets
	double xa = -2.*Pi/3.;
	double xb = 0;
	double xc = 2.*Pi/3.;
	double Fa = func(xa);
	double Fb = func(xb);
	double Fc = func(xc);
	if (Fa < Fb && Fa < Fc) {
		xa = -4.*Pi/3.;
		xb = -2.*Pi/3.;
		xc = 0;
	} else if (Fc < Fa && Fc < Fb) {
		xa = 0;
		xb = 2.*Pi/3.;
		xc = 4.*Pi/3.;
	}

	// Setup minimizer
	Dbrent dbrent;
	dbrent.bracket(xa,xc,func);

	// Find minimum
	double sol;
	double xminA;
	try {
		xminA = dbrent.minimize(func);
	} catch(Exception& e) {
		RW_THROW("min failed1");
	}
	sol = xminA;

	// Find amount of shift required
	double shift = func(xminA);

	// Find polynomial for finding the roots of the original function
	Polynomial<> P(4);
	P[4] = (A-B)*(A-B)+C*C;
	P[3] = 2*((A-B)*D+C*E);
	P[2] = D*D+2*(A-B)*(B+F-shift)-C*C+E*E;
	P[1] = 2*(B+F-shift)*D-2*C*E;
	P[0] = (B+F-shift)*(B+F-shift)-E*E;

	// Deflate the polynomial be removing the new double root in xmin
	Polynomial<> Pdeflated = P.deflate(xminA);
	Pdeflated = Pdeflated.deflate(xminA);

	// Solve the remaining 2nd order polynomial (if it can be solved)
	if (4*P[2]*P[0] < P[1]*P[1]) {
		Dbrent dbrentb;
		double sqt = sqrt(P[1]*P[1]-4*P[2]*P[0]);
		double xBracketA = (-P[1]+sqt)/(2*P[2]);
		double xBracketB = (-P[1]-sqt)/(2*P[2]);
		dbrentb.bracket(xBracketA,xBracketB,func);

		// Find global minimum
		double xminB;
		try {
			xminB = dbrentb.minimize(func);
		} catch(Exception& e) {
			RW_THROW("min failed2");
		}
		sol = xminB;
	}

	res.second = r2*u*cos(sol)+r2*v*sin(sol)+cCircle2;
	res.first = cCircle1+normalize(res.second-cCircle1-dot(res.second-cCircle1,n1)*n1)*r1;

	return res;
}

std::vector<std::pair<Vector3D<>, Vector3D<> > > GeometricUtil::closestPointsOnCircleToCircle(const Vector3D<> &cCircle1, const Vector3D<> &n1, double r1, const Vector3D<> &cCircle2, const Vector3D<> &n2, double r2) {
	const static double TOLERANCE = std::numeric_limits<double>::epsilon();

	std::vector<std::pair<Vector3D<>, Vector3D<> > > res;

	const Vector3D<> u = getPerpendicular(n2);
	const Vector3D<> v = normalize(cross(n2,u));

	const double c1u = dot(cCircle1,u);
	const double c1v = dot(cCircle1,v);
	const double c2u = dot(cCircle2,u);
	const double c2v = dot(cCircle2,v);
	const double n1u = dot(n1,u);
	const double n1v = dot(n1,v);
	const double n1c1 = dot(n1,cCircle1);
	const double n1c2 = dot(n1,cCircle2);
	const double c1c1 = dot(cCircle1,cCircle1);
	const double c2c2 = dot(cCircle2,cCircle2);
	const double c1c2 = dot(cCircle1,cCircle2);
	const double r1r2 = r1*r2;

	// The following are normalized with r1*r2 to remove dimensionality
	const double a9 = 2*(c2u-c1u)/r1;
	const double a8 = 2*(c2v-c1v)/r1;
	const double a7 = r1/r2+r2/r1+(c1c1+c2c2-2*c1c2)/r1r2;
	const double a6 = -2*sqrt(r1/r2);
	const double a5 = -(n1u)*(n1u)*r2/r1;
	const double a4 = -(n1v)*(n1v)*r2/r1;
	const double a3 = 2*(c2u-c1u-(n1c2-n1c1)*n1u)/r2;
	const double a2 = 2*(c2v-c1v-(n1c2-n1c1)*n1v)/r1;
	const double a1 = -2*r2/r1*n1u*n1v;
	const double a0 = r2/r1+(c1c1+c2c2-2*c1c2-(n1c2-n1c1)*(n1c2-n1c1))/r1r2;
	//std::cout << "f[\\[Theta]_]:=" << a9 << " Cos[\\[Theta]]+" << a8 << " Sin[\\[Theta]]+" << a7 <<"+" << a6 << "Sqrt[" << a5 << " Cos[\\[Theta]]^2+" << a4 << " Sin[\\[Theta]]^2+" << a3 << " Cos[\\[Theta]]+" << a2 << " Sin[\\[Theta]]+" << a1 << " Cos[\\[Theta]]Sin[\\[Theta]]+" << a0 << "];" << std::endl;

	// Distance functions
	const FuncCircleCircleDist dist(a9,a8,a7,a6,a5,a4,a3,a2,a1,a0);
	const FuncCircleCircleDist distNeg(-a9,-a8,-a7,-a6,a5,a4,a3,a2,a1,a0);
	//long begin, end;

	// Find one min and one max
	std::vector<double> solMin;
	std::vector<double> solMax;
	//begin = rw::common::TimerUtil::currentTimeUs();
	{
		// Find brackets
		double xa = -2.*Pi/3.;
		double xb = 0;
		double xc = 2.*Pi/3.;
		const double Fa = dist(xa);
		const double Fb = dist(xb);
		const double Fc = dist(xc);
		if (Fa < Fb && Fa < Fc) {
			xa = -4.*Pi/3.;
			xb = -2.*Pi/3.;
			xc = 0;
		} else if (Fc < Fa && Fc < Fb) {
			xa = 0;
			xb = 2.*Pi/3.;
			xc = 4.*Pi/3.;
		}

		// Setup minimizer
		Dbrent dbrent(TOLERANCE);
		dbrent.ax = xa;
		dbrent.bx = xb;
		dbrent.cx = xc;
		dbrent.fa = Fa;
		dbrent.fb = Fb;
		dbrent.fc = Fc;

		// Find minimum
		try {
			solMin.push_back(dbrent.minimize(dist));
			//std::cout << "solMin=" << solMin.back() << ";" << std::endl;
		} catch(Exception& e) {
			RW_THROW("min failed3");
		}
	}
	//end = rw::common::TimerUtil::currentTimeUs();
	//std::cout << "time for min: " << end - begin << std::endl;

	{
		// Find brackets
		double xa = -2.*Pi/3.;
		double xb = 0;
		double xc = 2.*Pi/3.;
		const double Fa = distNeg(xa);
		const double Fb = distNeg(xb);
		const double Fc = distNeg(xc);
		if (Fa < Fb && Fa < Fc) {
			xa = -4.*Pi/3.;
			xb = -2.*Pi/3.;
			xc = 0;
		} else if (Fc < Fa && Fc < Fb) {
			xa = 0;
			xb = 2.*Pi/3.;
			xc = 4.*Pi/3.;
		}

		// Setup minimizer
		Dbrent dbrent(TOLERANCE);
		dbrent.ax = xa;
		dbrent.bx = xb;
		dbrent.cx = xc;
		dbrent.fa = Fa;
		dbrent.fb = Fb;
		dbrent.fc = Fc;

		// Find maximum
		try {
			solMax.push_back(dbrent.minimize(distNeg));
			//std::cout << "solMax=" << solMax.back() << ";" << std::endl;
		} catch(Exception& e) {
			RW_THROW("min failed4");
		}
	}

	// If one of the found extremums are local, the global extremum can also be found.
	// First setup polynomials used to find brackets
	Polynomial<> Pmin(4);
	Polynomial<> Pmax(4);
	{
		const double shiftMin = dist(solMin.back());
		const double shiftMax = distNeg(solMax.back());

		// Trigonometric equation for finding thetas where f(theta)=0
		const double A = a9*a9-a5*a6*a6;
		const double B = a8*a8-a4*a6*a6;
		const double C = 2*a8*a9-a1*a6*a6;
		const double D = 2*a7*a9-a3*a6*a6;
		const double E = 2*a7*a8-a2*a6*a6;
		const double Fmin = a7*a7+(a0-shiftMin)*a6*a6;
		const double Fmax = a7*a7+(a0-shiftMax)*a6*a6;

		Polynomial<> PminTmp(4);
		Polynomial<> PmaxTmp(4);
		PminTmp[4] = (A-B)*(A-B)+C*C;
		PminTmp[3] = 2*((A-B)*D+C*E);
		PminTmp[2] = 2*(A-B)*(B+Fmin)+E*E-C*C;
		PminTmp[1] = 2*(B+Fmin)*D-2*C*E+D*D;
		PminTmp[0] = (B+Fmin)*(B+Fmin)-E*E;
		PmaxTmp[4] = PminTmp[4];
		PmaxTmp[3] = PminTmp[3];
		PmaxTmp[2] = 2*(A-B)*(B+Fmax)+E*E-C*C;
		PmaxTmp[1] = 2*(B+Fmax)*D-2*C*E+D*D;
		PmaxTmp[0] = (B+Fmax)*(B+Fmax)-E*E;

		// Deflate the polynomials as two solutions are already known
		PminTmp = PminTmp.deflate(cos(solMin.back()));
		PminTmp = PminTmp.deflate(cos(solMax.back()));
		Pmin = PminTmp;
		PmaxTmp = PmaxTmp.deflate(cos(solMin.back()));
		PmaxTmp = PmaxTmp.deflate(cos(solMax.back()));
		Pmax = PmaxTmp;
	}
	RW_ASSERT(Pmin.order() == 2);
	RW_ASSERT(Pmax.order() == 2);

	if (4*Pmin[2]*Pmin[0] < Pmin[1]*Pmin[1]) {
		// Two brackets are determined for the global minimum
		const double sqt = sqrt(Pmin[1]*Pmin[1]-4*Pmin[2]*Pmin[0]);
		const double t1 = (-Pmin[1]+sqt)/(2*Pmin[2]);
		const double t2 = (-Pmin[1]-sqt)/(2*Pmin[2]);

		double bracket1;
		double val1;
		if (fabs(t1) <= 1.) {
			const double sol1A = +acos(t1);
			const double sol1B = -acos(t1);
			const double valA = dist(sol1A);
			const double valB = dist(sol1B);
			if (fabs(valA) < fabs(valB)) {
				bracket1 = sol1A;
				val1 = valA;
			} else {
				bracket1 = sol1B;
				val1 = valB;
			}
			RW_ASSERT(!std::isnan(bracket1));
		}

		double bracket2;
		double val2;
		if (fabs(t2) <= 1.) {
			const double sol2A = +acos(t2);
			const double sol2B = -acos(t2);
			const double valA = dist(sol2A);
			const double valB = dist(sol2B);
			if (fabs(valA) < fabs(valB)) {
				bracket2 = sol2A;
				val2 = valA;
			} else {
				bracket2 = sol2B;
				val2 = valB;
			}
			RW_ASSERT(!std::isnan(bracket2));
		}
		//std::cout << " - bracket1=" << bracket1 << ";" << std::endl;
		//std::cout << " - bracket2=" << bracket2 << ";" << std::endl;

		// Find brackets
		double xa = -2.*Pi/3.;
		double xb = 0;
		double xc = 2.*Pi/3.;
		double Fa = dist(xa);
		double Fb = dist(xb);
		double Fc = dist(xc);
		if (bracket1 < bracket2) {
			xa = bracket1;
			xb = (bracket1+bracket2)/2;
			xc = bracket2;
			Fa = val1;
			Fb = dist(xb);
			Fc = val2;
		} else {
			xa = bracket2;
			xb = (bracket1+bracket2)/2;
			xc = bracket1;
			Fa = val2;
			Fb = dist(xb);
			Fc = val1;
		}

		// Setup minimizer
		Dbrent dbrent(TOLERANCE);
		dbrent.ax = xa;
		dbrent.bx = xb;
		dbrent.cx = xc;
		dbrent.fa = Fa;
		dbrent.fb = Fb;
		dbrent.fc = Fc;

		// Find minimum
		try {
			solMin.push_back(dbrent.minimize(dist));
			//std::cout << "solMin2=" << solMin.back() << ";" << std::endl;
		} catch(Exception& e) {
			RW_THROW("min failed3");
		}
	} else if (4*Pmin[2]*Pmin[0] == Pmin[1]*Pmin[1]) {
		// The solution is the global minimum
		const double t = -Pmin[1]/(2*Pmin[2]);
		if (fabs(t) <= 1.) {
			const double solA = +acos(t);
			const double solB = -acos(t);

			if (fabs(dist.df(solA)) < fabs(dist.df(solB)))
				solMin.push_back(solA);
			else
				solMin.push_back(solB);
			//std::cout << "solMin3=" << solMin.back() << ";" << std::endl;
			RW_ASSERT(!std::isnan(solMin.back()));
		}
	}

	if (solMin.size() == 1) {
		// We have the global minimum, but not the other local one (if there is two minimums)
		// We make 8th degree polynomial to determine this
		Polynomial<> dfZeroPol = dist.getDfZeroPol();
		// We deflate the polynomial with the solutions we already know
		dfZeroPol = dfZeroPol.deflate(std::cos(solMin[0]));
		for (std::size_t i = 0; i < solMax.size(); i++) {
			dfZeroPol = dfZeroPol.deflate(std::cos(solMax[i]));
		}
		//std::cout << "zero pol: " << dfZeroPol << std::endl;
		std::vector<double> coef;
		for (int i = (int)dfZeroPol.order(); i >= 0; i--) {
			coef.push_back(dfZeroPol[i]);
		}
		PolynomialSolver solver(coef);
		const std::vector<double> sols = solver.getRealSolutions();
		//std::cout << "sols: ";
		//for (std::size_t i = 0; i < sols.size(); i++)
		//	std::cout << " " << sols[i];
		//std::cout << std::endl;

		// Now we have up to 6 new solutions where some are false roots
		// First we find the candidate solutions
		std::vector<double> candidates;
		for (std::size_t i = 0; i < sols.size(); i++) {
			const double t = sols[i];
			if (fabs(t) <= 1.) {
				const double solA = +acos(t);
				const double solB = -acos(t);

				if (!dist.singular(solA)) {
					//std::cout << "test: " << solA << " " << dist.df(solA) << " " << dist.ddf(solA) << std::endl;
					if (dist.ddf(solA) > 0) {
						if (fabs(dist.df(solA)) < 1e-8) {
							candidates.push_back(solA);
							RW_ASSERT(!std::isnan(candidates.back()));
						}
					}
				}
				if (!dist.singular(solB)) {
					//std::cout << "test: " << solB << " " << dist.df(solB) << " " << dist.ddf(solB) << std::endl;
					if (dist.ddf(solB) > 0) {
						if (fabs(dist.df(solB)) < 1e-8) {
							candidates.push_back(solB);
							RW_ASSERT(!std::isnan(candidates.back()));
						}
					}
				}
			}
		}
		//std::cout << "candidates " << candidates.size() << std::endl;
		RW_ASSERT(candidates.size() <= 2);
		if (candidates.size() == 1) {
			//std::cout << "Added one Laguerre solution: " << candidates[0] << " dif: " << fabs(solMin[0])-fabs(candidates[0]) << std::endl;
			solMin.push_back(candidates[0]);
		} else if (candidates.size() == 2) {
			if (fabs(candidates[0]-solMin[0]) > fabs(candidates[1]-solMin[0])) {
				//std::cout << "Added one of two Laguerre solutions: " << candidates[0] << " dif: " << fabs(solMin[0])-fabs(candidates[0]) << std::endl;
				solMin.push_back(candidates[0]);
			} else {
				//std::cout << "Added one of two Laguerre solutions: " << candidates[1] << " dif: " << fabs(solMin[1])-fabs(candidates[0]) << std::endl;
				solMin.push_back(candidates[1]);
			}
		}
	}

	res.resize(solMin.size());
	for (std::size_t i = 0; i < solMin.size(); i++) {
		res[i].second = r2*u*cos(solMin[i])+r2*v*sin(solMin[i])+cCircle2;
		res[i].first = closestPointOnCircleToPoint(cCircle1, r1, n1, res[i].second);
	}
	return res;
}

GeometricUtil::PointPair GeometricUtil::extremalPointsOnEllipseToPoint(const Vector3D<> &point, const Vector3D<> &centerEllipse, const Vector3D<> &dir1, double r1, const Vector3D<> &dir2, double r2) {
	const static double TOLERANCE = std::numeric_limits<double>::epsilon();

	PointPair res;
	Vector3D<>& max = res.first;
	Vector3D<>& min = res.second;

	const double A = r1*r1;
	const double B = r2*r2;
	const double C = 0;
	const double D = 2*r1*dot(dir1,centerEllipse-point);
	const double E = 2*r2*dot(dir2,centerEllipse-point);
	const double F = dot(centerEllipse,centerEllipse-point);

	// Minimization
	{
		// Setup function
		FuncdTrig func(A, B, C, D, E, F);

		// Find brackets
		double xa = -2.*Pi/3.;
		double xb = 0;
		double xc = 2.*Pi/3.;
		double Fa = func(xa);
		double Fb = func(xb);
		double Fc = func(xc);
		if (Fa < Fb && Fa < Fc) {
			xa = -4.*Pi/3.;
			xb = -2.*Pi/3.;
			xc = 0;
		} else if (Fc < Fa && Fc < Fb) {
			xa = 0;
			xb = 2.*Pi/3.;
			xc = 4.*Pi/3.;
		}

		// Setup minimizer
		Dbrent dbrent(TOLERANCE);
		dbrent.ax = xa;
		dbrent.bx = xb;
		dbrent.cx = xc;
		dbrent.fa = Fa;
		dbrent.fb = Fb;
		dbrent.fc = Fc;
		//dbrent.bracket(xa,xc,func);

		// Find minimum
		double sol;
		double xminA;
		try {
			xminA = dbrent.minimize(func);
		} catch(Exception& e) {
			RW_THROW("min failed5");
		}
		sol = xminA;

		// Find amount of shift required
		double shift = func(xminA);

		// Find polynomial for finding the roots of the original function
		Polynomial<> P(4);
		P[4] = (A-B)*(A-B);
		P[3] = 2*(A-B)*D;
		P[2] = D*D+E*E+2*(A-B)*(B+F-shift);
		P[1] = 2*(B+F-shift)*D;
		P[0] = (B+F-shift)*(B+F-shift)-E*E;

		// Deflate the polynomial be removing the new double root in xmin
		Polynomial<> Pdeflated = P.deflate(xminA);
		Pdeflated = Pdeflated.deflate(xminA);

		// Solve the remaining 2nd order polynomial (if it can be solved)
		if (4*P[2]*P[0] <= P[1]*P[1]) {
			Dbrent dbrentb(TOLERANCE);
			double sqt = sqrt(P[1]*P[1]-4*P[2]*P[0]);
			double xBracketA = (-P[1]+sqt)/(2*P[2]);
			double xBracketB = (-P[1]-sqt)/(2*P[2]);
			dbrentb.bracket(xBracketA,xBracketB,func);

			// Find global minimum
			double xminB;
			try {
				xminB = dbrent.minimize(func);
			} catch(Exception& e) {
				RW_THROW("min failed6");
			}
			sol = xminB;
		}
		min = r1*dir1*cos(sol)+r2*dir2*sin(sol)+centerEllipse;
	}

	// Maximization
	{
		// Setup function
		FuncdTrig func(-A, -B, -C, -D, -E, -F);

		// Find brackets
		double xa = -2.*Pi/3.;
		double xb = 0;
		double xc = 2.*Pi/3.;
		double Fa = func(xa);
		double Fb = func(xb);
		double Fc = func(xc);
		if (Fa < Fb && Fa < Fc) {
			xa = -4.*Pi/3.;
			xb = -2.*Pi/3.;
			xc = 0;
		} else if (Fc < Fa && Fc < Fb) {
			xa = 0;
			xb = 2.*Pi/3.;
			xc = 4.*Pi/3.;
		}

		// Setup minimizer
		Dbrent dbrent(TOLERANCE);
		dbrent.ax = xa;
		dbrent.bx = xb;
		dbrent.cx = xc;
		dbrent.fa = Fa;
		dbrent.fb = Fb;
		dbrent.fc = Fc;
		//dbrent.bracket(xa,xc,func);

		// Find minimum
		double sol;
		double xminA;
		try {
			xminA = dbrent.minimize(func);
		} catch(Exception& e) {
			RW_THROW("min failed7");
		}
		sol = xminA;

		// Find amount of shift required
		double shift = func(xminA);

		// Find polynomial for finding the roots of the original function
		Polynomial<> P(4);
		P[4] = (A-B)*(A-B);
		P[3] = 2*(A-B)*D;
		P[2] = D*D+E*E+2*(A-B)*(B+F-shift);
		P[1] = 2*(B+F-shift)*D;
		P[0] = (B+F-shift)*(B+F-shift)-E*E;

		// Deflate the polynomial be removing the new double root in xmin
		Polynomial<> Pdeflated = P.deflate(xminA);
		Pdeflated = Pdeflated.deflate(xminA);

		// Solve the remaining 2nd order polynomial (if it can be solved)
		if (4*P[2]*P[0] <= P[1]*P[1]) {
			Dbrent dbrentb(TOLERANCE);
			double sqt = sqrt(P[1]*P[1]-4*P[2]*P[0]);
			double xBracketA = (-P[1]+sqt)/(2*P[2]);
			double xBracketB = (-P[1]-sqt)/(2*P[2]);
			dbrentb.bracket(xBracketA,xBracketB,func);

			// Find global minimum
			double xminB;
			try {
				xminB = dbrent.minimize(func);
			} catch(Exception& e) {
				RW_THROW("min failed8");
			}
			sol = xminB;
		}
		max = r1*dir1*cos(sol)+r2*dir2*sin(sol)+centerEllipse;
	}

	return res;
}

std::vector<rw::math::Vector3D<> > GeometricUtil::intersectCircleEllipse(const rw::math::Vector3D<> &normal, const rw::math::Vector3D<> &centerCircle, double rc, const rw::math::Vector3D<> &centerEllipse, const rw::math::Vector3D<> &dir1, double r1, const rw::math::Vector3D<> &dir2, double r2) {
	std::vector<Vector3D<> > res;

	const double uu = dot(r1*dir1,r1*dir1);
	const double vv = dot(r2*dir2,r2*dir2);
	const Vector3D<> cp = centerEllipse-centerCircle;

	const double Ka = uu-vv;
	const double Kb = rc-vv-dot(cp,cp);

	const double c = 2*dot(cp,r1*dir1);
	const double d = 2*dot(cp,r2*dir2);
	const double dd = d*d;

	const double A = Ka*Ka;
	const double B = 2*Ka;
	const double C = c*c+dd-2*Ka*Kb;
	const double D = -2*c*Kb;
	const double E = Kb*Kb-dd;

	std::vector<double> coef;
	coef.push_back(A);
	coef.push_back(B);
	coef.push_back(C);
	coef.push_back(D);
	coef.push_back(E);
	PolynomialSolver solver(coef);
	std::vector<double> sol = solver.getRealSolutions();

	for (std::vector<double>::iterator it = sol.begin(); it < sol.end(); it++) {
		if (*it >= -1. && *it <= 1.) {
			double theta1 = +acos(*it);
			double theta2 = -acos(*it);
			double eval1 = uu*(*it)+vv*sin(theta1)+c*(*it)+d*sin(theta1)+dot(cp,cp)-rc;
			double eval2 = uu*(*it)+vv*sin(theta2)+c*(*it)+d*sin(theta2)+dot(cp,cp)-rc;
			if (fabs(eval1) <= fabs(eval2)) {
				Vector3D<> p1 = r1*dir1*(*it)+r2*dir2*sin(theta1)+centerEllipse;
				res.push_back(p1);
			} else {
				Vector3D<> p2 = r1*dir1*(*it)+r2*dir2*sin(theta2)+centerEllipse;
				res.push_back(p2);
			}
		}
	}

	return res;
}
