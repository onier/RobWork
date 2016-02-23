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

#include "GradientProjectionMethod.hpp"

#include <rw/common/macros.hpp>
#include <vector>
#include <set>

using namespace rwlibs::algorithms;

struct GradientProjectionMethod::TimeCompare {
    bool operator()(const std::pair<Eigen::VectorXd::Index, double>& firstElem, const std::pair<Eigen::VectorXd::Index, double>& secondElem) const {
    	return firstElem.second < secondElem.second;
    }
};

GradientProjectionMethod::GradientProjectionMethod() {
}

GradientProjectionMethod::~GradientProjectionMethod() {
}

void GradientProjectionMethod::minimize(const QuadraticFunction& f, const Eigen::VectorXd& l, const Eigen::VectorXd& u, Eigen::VectorXd& x, const std::size_t iterations, const double eps) const {
	const Eigen::MatrixXd& G = f.A();
	const Eigen::VectorXd& c = f.b();
	const Eigen::VectorXd::Index N = G.rows();
	RW_ASSERT(G.cols() == N);
	RW_ASSERT(c.rows() == N);
	RW_ASSERT(l.rows() == N);
	RW_ASSERT(u.rows() == N);
	std::vector<std::pair<Eigen::VectorXd::Index, double> > tmax(N);
	for (std::size_t i = 0; i < (std::size_t) N; i++) {
		tmax[i].first = i;
		tmax[i].second = positiveInfinity();
	}
	double ch = 1;
	for(std::size_t i = 0; i <= iterations && ch > eps; i++) {
		std::cout << "iteration " << i << " " << x.transpose() << std::endl;
		// Initialize iteration
		const Eigen::VectorXd xold = x;
		//const Eigen::VectorXd g = G*x+c;
		Eigen::VectorXd d = -G*x-c;
		std::vector<std::pair<Eigen::VectorXd::Index, double> > t = tmax;
		// Calculation of where constraints are violated
		for (std::size_t j = 0; j < (std::size_t) N; j++) {
			std::cout << "checking constraint " << j << std::endl;
			if (l[j] != negativeInfinity() && d[j] < 0) {
				std::cout << "lower bound" << std::endl;
				if (x[j]-l[j] < eps && d[j] < -eps) {
					t[j].second = 0;
					std::cout << "time zero" << std::endl;
				} else if (d[j] < -eps) {
					t[j].second = std::fabs(-(x[j]-l[j])/d[j]);
				}
			} else if (u[j] != positiveInfinity() && d[j] > 0) {
				if (u[j]-x[j] < eps && d[j] > eps)
					t[j].second = 0;
				else if (d[j] > eps)
					t[j].second = std::fabs(-(x[j]-u[j])/d[j]);
			}
		}
		// Sort & Remove Duplicates
		std::set<std::pair<Eigen::VectorXd::Index, double>, TimeCompare> ts(t.begin(),t.end());
		// Find Cauchy point
		bool xCN = true;
		double tj = 0;
		std::set<std::pair<Eigen::VectorXd::Index, double>, TimeCompare>::const_iterator it;
		for (it = ts.begin(); it != ts.end() && xCN; it++) {
			std::cout << "interval " << tj << " to " << it->second << std::endl;
			const double dtj = it->second-tj;
			//const double f = 0.5*x.dot(G*x)+c.dot(x);
			const double fp = x.dot(G*d)+c.dot(d);
			const double fpp = d.dot(G*d);
			if (fp > 0) {
				//xC = x;
				xCN = false;
			}
			const double dt = -fp/fpp;
			if (dt < dtj) {
				x += dt*d;
				//xC = x;
				xCN = false;
			}
			if (xCN) {
				tj += dtj;
				x += dtj*d;
				d[it->first] = 0;
			}
		}
		// Subspace minimization
		/*Eigen::MatrixXd neg = Eigen::MatrixXd::Identity(N,N);
		for (Eigen::MatrixXd::Index j = 0; j < N; j++) {
			if (x[j] > eps || (G*x+c)[j] < -eps) {
				neg(j,j) = 1;
			} else {
				neg(j,j) = 0;
			}
		}
		const Eigen::MatrixXd Gsub1 = G*neg;
		const Eigen::MatrixXd Gsub2 = neg*Gsub1;
		const Eigen::VectorXd cSub = neg*c - Gsub1.transpose()*x;
		const Eigen::VectorXd dx = neg*yy;
		std::list<double> dxv;
		//for (std::size_t j = 1; j < (std::size_t) N; j++) {
			//if (dx[j] == y[j]) {
				//dxv.push_back(y[j]);
			//}
		//}
		//double res = minimize(0.5*(x+dx)*G*(x+dx)+c*(x+dx),dxv); // Symbolically in y
		//const Eigen::VectorXd dx1 = dx(res);
		const Eigen::VectorXd dx1(N);
		// Find how far we can go in subspace minimum without exceeding the limits
		double alp = 1;
		for (std::size_t j = 1; j < (std::size_t) N; j++) {
			if (neg(j,j) == 1) {
				alp = std::min(alp,std::fabs(x[j]/(eps*eps+std::fabs(dx1[j]))));
			}
		}
		x += alp*dx1;
		*/
		ch = (xold-x).squaredNorm();
	}
}

double GradientProjectionMethod::negativeInfinity() {
	const static double val = std::numeric_limits<double>::min();
	return val;
}

double GradientProjectionMethod::positiveInfinity() {
	const static double val = std::numeric_limits<double>::max();
	return val;
}
