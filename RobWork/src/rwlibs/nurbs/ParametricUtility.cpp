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

#include "ParametricUtility.hpp"

using namespace rw::math;
using namespace rwlibs::nurbs;

double ParametricUtility::bernstein(std::size_t i, std::size_t degree, double t) {
	if (degree == 0)
		return 1;
	else
		return bernstein(i,degree-1,t)*(1.-t)+bernstein(i-1,degree-1,t)*t;
}

double ParametricUtility::binomial(unsigned int n, unsigned int k) {
	if (k < 0 || k > n)
		return 0;
	double bin = 1;
	for (unsigned int i = 1; i <= k; i++) {
		bin = bin*((double)(n-(k-i)));
		bin = bin/((double)i);
	}
	return bin;
}

Vector3D<> ParametricUtility::toVector3D(const VectorND<3> &vec) {
	return Vector3D<>(vec[0],vec[1],vec[2]);
}

std::vector<Vector3D<> > ParametricUtility::toVector3D(const std::vector<VectorND<3> > &vec) {
	std::vector<Vector3D<> > newVec;
	for (std::size_t i = 0; i < vec.size(); i++)
		newVec.push_back(toVector3D(vec[i]));
	return newVec;
}

std::vector<std::vector<Vector3D<> > > ParametricUtility::toVector3D(const std::vector<std::vector<VectorND<3> > > &vec) {
	std::vector<std::vector<Vector3D<> > > newVec;
	for (std::size_t i = 0; i < vec.size(); i++) {
		std::vector<Vector3D<> > v;
		for (std::size_t j = 0; j < vec[i].size(); j++) {
			v.push_back(toVector3D(vec[i][j]));
		}
		newVec.push_back(v);
	}
	return newVec;
}

std::vector<VectorND<1> > ParametricUtility::toVectorND(const std::vector<double> &vec) {
	std::vector<VectorND<1> > res;
	for (std::size_t i = 0; i < vec.size(); i++) {
		VectorND<1> v;
		v[0] = vec[i];
		res.push_back(v);
	}
	return res;
}

VectorND<3> ParametricUtility::toVectorND(const Vector3D<> &vec) {
	VectorND<3> newVec(vec.m());
	return newVec;
}

std::vector<VectorND<3> > ParametricUtility::toVectorND(const std::vector<Vector3D<> > &vec) {
	std::vector<VectorND<3> > newVec;
	for (std::size_t i = 0; i < vec.size(); i++)
		newVec.push_back(toVectorND(vec[i]));
	return newVec;
}

std::vector<std::vector<VectorND<3> > > ParametricUtility::toVectorND(const std::vector<std::vector<Vector3D<> > > &vec) {
	std::vector<std::vector<VectorND<3> > > newVec;
	for (std::size_t i = 0; i < vec.size(); i++) {
		std::vector<VectorND<3> > v;
		for (std::size_t j = 0; j < vec[i].size(); j++) {
			v.push_back(toVectorND(vec[i][j]));
		}
		newVec.push_back(v);
	}
	return newVec;
}
