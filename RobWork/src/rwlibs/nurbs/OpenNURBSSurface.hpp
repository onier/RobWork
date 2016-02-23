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

#ifndef RWLIBS_NURBS_OPENNURBSSURFACE_HPP_
#define RWLIBS_NURBS_OPENNURBSSURFACE_HPP_

/**
 * @file OpenNURBSSurface.hpp
 *
 * \copydoc rwlibs::nurbs::OpenNURBSSurface
 */

#include "ParametricSurface.hpp"
#include "NURBSPatch.hpp"
#include <rw/geometry/Box.hpp>
#include <rw/geometry/Cylinder.hpp>

namespace rwlibs {
namespace nurbs {
//! @addtogroup nurbs
//! @{
class OpenNURBSSurface: public rwlibs::nurbs::ParametricSurface {
public:
    //! @brief smart pointer type to this class
	typedef rw::common::Ptr<OpenNURBSSurface> Ptr;

	//! @copydoc GeometryData::getType
	GeometryType getType() const;

	//! @copydoc GeometryData::getTriMesh
	rw::common::Ptr<rw::geometry::TriMesh> getTriMesh(bool forceCopy=true);

	OpenNURBSSurface();
	OpenNURBSSurface(rw::geometry::Box box);
	OpenNURBSSurface(rw::geometry::Cylinder cylinder);
	~OpenNURBSSurface();
	void addPatch(NURBSPatch::Ptr patch);
	std::vector<NURBSPatch::Ptr> getPatches();

	void save3DM(const std::string & file) const;
	static OpenNURBSSurface::Ptr load3DM(const std::string &file);

private:
	std::vector<NURBSPatch::Ptr> _patches;
};
//! @}
} /* namespace nurbs */
} /* namespace rwlibs */
#endif /* RWLIBS_NURBS_OPENNURBSSURFACE_HPP_ */
