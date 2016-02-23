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

#ifndef RWLIBS_NURBS_PARAMETRICUTILITY_HPP_
#define RWLIBS_NURBS_PARAMETRICUTILITY_HPP_

/**
 * @file ParametricUtility.hpp
 *
 * \copydoc rwlibs::nurbs::ParametricUtility
 */

#include <rw/math/Vector3D.hpp>
#include <rw/math/VectorND.hpp>

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class ParametricUtility {
public:
	static double bernstein(std::size_t i, std::size_t degree, double t);
	static double binomial(unsigned int n, unsigned int k);

	template <std::size_t N>
	static rw::math::VectorND<N+1> toHomogeneous(const rw::math::VectorND<N> &vec) {
		rw::math::VectorND<N+1> pH;
		for (std::size_t j = 0; j < N; j++)
			pH[j] = vec[j];
		pH[N] = 1;
		return pH;
	}

	template <std::size_t N>
	static rw::math::VectorND<N+1> toHomogeneous(const rw::math::VectorND<N> &vec, double weight) {
		rw::math::VectorND<N+1> pH;
		for (std::size_t j = 0; j < N; j++)
			pH[j] = vec[j]*weight;
		pH[N] = weight;
		return pH;
	}

	template <std::size_t N>
	static std::vector<rw::math::VectorND<N+1> > toHomogeneous(const std::vector<rw::math::VectorND<N> > &vec, std::vector<double> weights = std::vector<double>()) {
		std::vector<rw::math::VectorND<N+1> > pHs;
		for (std::size_t i = 0; i < vec.size(); i++) {
			if (weights.size() > 0)
				pHs.push_back(toHomogeneous(vec[i],weights[i]));
			else
				pHs.push_back(toHomogeneous(vec[i]));
		}
		return pHs;
	}

	template <std::size_t N>
	static std::vector<std::vector<rw::math::VectorND<N+1> > > toHomogeneous(const std::vector<std::vector<rw::math::VectorND<N> > > &vec, std::vector<std::vector<double> > weights = std::vector<std::vector<double> >()) {
		std::vector<std::vector<rw::math::VectorND<N+1> > > pHs;
		for (std::size_t i = 0; i < vec.size(); i++) {
			std::vector<rw::math::VectorND<N+1> > p;
			for (std::size_t j = 0; j < vec[i].size(); j++) {
				if (weights.size() > 0)
					p.push_back(toHomogeneous(vec[i][j],weights[i][j]));
				else
					p.push_back(toHomogeneous(vec[i][j]));
			}
			pHs.push_back(p);
		}
		return pHs;
	}

	template <std::size_t N>
	static rw::math::VectorND<N> fromHomogeneous(const rw::math::VectorND<N+1> &vec) {
		rw::math::VectorND<N> p;
		for (std::size_t j = 0; j < N; j++)
			p[j] = vec[j]/vec[N];
		return p;
	}

	template <std::size_t N>
	static std::vector<rw::math::VectorND<N> > fromHomogeneous(const std::vector<rw::math::VectorND<N+1> > &vec) {
		std::vector<rw::math::VectorND<N> > ps;
		for (std::size_t i = 0; i < vec.size(); i++)
			ps.push_back(fromHomogeneous<N>(vec[i]));
		return ps;
	}

	template <std::size_t N>
	static std::vector<std::vector<rw::math::VectorND<N> > > fromHomogeneous(const std::vector<std::vector<rw::math::VectorND<N+1> > > &vec) {
		std::vector<std::vector<rw::math::VectorND<N> > > ps;
		for (std::size_t i = 0; i < vec.size(); i++) {
			std::vector<rw::math::VectorND<N> > p;
			for (std::size_t j = 0; j < vec[i].size(); j++)
				p.push_back(fromHomogeneous<N>(vec[i][j]));
			ps.push_back(p);
		}
		return ps;
	}

	template <std::size_t N>
	static double getWeights(const rw::math::VectorND<N+1> &vec) {
		return vec[N];
	}

	template <std::size_t N>
	static std::vector<double> getWeights(const std::vector<rw::math::VectorND<N+1> > &vec) {
		std::vector<double> w;
		for (std::size_t i = 0; i < vec.size(); i++)
			w.push_back(getWeights<N>(vec[i]));
		return w;
	}

	template <std::size_t N>
	static std::vector<std::vector<double> > getWeights(const std::vector<std::vector<rw::math::VectorND<N+1> > > &vec) {
		std::vector<std::vector<double> > ws;
		for (std::size_t i = 0; i < vec.size(); i++) {
			std::vector<double> w;
			for (std::size_t j = 0; j < vec[i].size(); j++)
				w.push_back(getWeights<N>(vec[i][j]));
			ws.push_back(w);
		}
		return ws;
	}

	static rw::math::Vector3D<> toVector3D(const rw::math::VectorND<3> &vec);
	static std::vector<rw::math::Vector3D<> > toVector3D(const std::vector<rw::math::VectorND<3> > &vec);
	static std::vector<std::vector<rw::math::Vector3D<> > > toVector3D(const std::vector<std::vector<rw::math::VectorND<3> > > &vec);

	static std::vector<rw::math::VectorND<1> > toVectorND(const std::vector<double> &vec);

	static rw::math::VectorND<3> toVectorND(const rw::math::Vector3D<> &vec);
	static std::vector<rw::math::VectorND<3> > toVectorND(const std::vector<rw::math::Vector3D<> > &vec);
	static std::vector<std::vector<rw::math::VectorND<3> > > toVectorND(const std::vector<std::vector<rw::math::Vector3D<> > > &vec);

private:
	ParametricUtility() {};
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_PARAMETRICUTILITY_HPP_ */
