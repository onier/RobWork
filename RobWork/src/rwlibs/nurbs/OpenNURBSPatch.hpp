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

#ifndef RWLIBS_NURBS_NURBSPATCH_HPP_
#define RWLIBS_NURBS_NURBSPATCH_HPP_

/**
 * @file OpenNURBSPatch.hpp
 *
 * \copydoc rwlibs::nurbs::NURBSPatch
 */

#include "ParametricPatch.hpp"

#include <utility>
#include <vector>
#include <rw/common.hpp>
#include <rw/math/Vector3D.hpp>
#include <opennurbs/src/opennurbs.h>

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class NURBSPatch: public ParametricPatch {
public:
    //! @brief smart pointer type to this class
	typedef rw::common::Ptr<NURBSPatch> Ptr;

	//! @copydoc GeometryData::getType
	GeometryType getType() const;

	//! @copydoc GeometryData::getTriMesh
	rw::common::Ptr<rw::geometry::TriMesh> getTriMesh(bool forceCopy=true);

	NURBSPatch(ON_NurbsSurface* surface);

	static NURBSPatch::Ptr make(
			unsigned int orderU,
			unsigned int orderV,
			std::vector<std::vector<rw::math::Vector3D<> > > controlPoints,
			std::vector<double> knotsU,
			std::vector<double> knotsV
	);

	static NURBSPatch::Ptr make(
			unsigned int orderU,
			unsigned int orderV,
			std::vector<std::vector<rw::math::Vector3D<> > > controlPoints,
			std::vector<double> knotsU,
			std::vector<double> knotsV,
			std::vector<std::vector<double> > weights
	);

	/**
	 * @brief destructor
	 */
	virtual ~NURBSPatch();

	std::pair<unsigned int,unsigned int> getDegree() const;
	std::pair<unsigned int,unsigned int> getOrder() const;
	std::vector<double> getKnotsU() const;
	std::vector<double> getKnotsV() const;
	std::vector<std::vector<rw::math::Vector3D<> > > getControlPoints() const;
	std::vector<std::vector<double> > getWeights() const;

	void save3DM(const std::string & file) const;
	static NURBSPatch::Ptr load3DM(const std::string &file);

	ON_NurbsSurface* getRaw();

private:
	static void validate(
			unsigned int orderU,
			unsigned int orderV,
			const std::vector<std::vector<rw::math::Vector3D<> > > &controlPoints,
			std::vector<double> &knotsU,
			std::vector<double> &knotsV
	);

	NURBSPatch(
			unsigned int orderU,
			unsigned int orderV,
			std::vector<std::vector<rw::math::Vector3D<> > > controlPoints,
			std::vector<double> knotsU,
			std::vector<double> knotsV
	);

	NURBSPatch(
			unsigned int orderU,
			unsigned int orderV,
			std::vector<std::vector<rw::math::Vector3D<> > > controlPoints,
			std::vector<double> knotsU,
			std::vector<double> knotsV,
			std::vector<std::vector<double> > weights
	);

	ON_NurbsSurface* _surface;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_NURBSPATCH_HPP_ */
