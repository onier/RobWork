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

#ifndef RWSIMLIBS_TEST_WORKCELLBUILDER_HPP_
#define RWSIMLIBS_TEST_WORKCELLBUILDER_HPP_

/**
 * @file WorkCellBuilder.hpp
 *
 * \copydoc rwsimlibs::test::WorkCellBuilder
 */

#include <rw/math/Vector3D.hpp>

namespace rw { namespace models { class WorkCell; } }

namespace rwsimlibs {
namespace test {
//! @addtogroup INSERT_DOC_GROUP

//! @{
/**
 * @brief INSERT_SHORT_DESCRIPTION
 */
class WorkCellBuilder {
public:
	struct ColorScheme {
		ColorScheme();
		ColorScheme(const rw::math::Vector3D<float>& color);
		ColorScheme(const rw::math::Vector3D<float>& fixedBodies, const rw::math::Vector3D<float>& kinematicBodies, const rw::math::Vector3D<float>& dynamicBodies);
		rw::math::Vector3D<float> fixedBodies;
		rw::math::Vector3D<float> kinematicBodies;
		rw::math::Vector3D<float> dynamicBodies;
	};

	struct PaHColors: public ColorScheme {
		PaHColors();
	};

	WorkCellBuilder(const ColorScheme& colors = PaHColors());
	virtual ~WorkCellBuilder();

	void addFloor(rw::common::Ptr<rw::models::WorkCell> wc, const std::string& name = "Floor", bool trimesh = false) const;
	void addPlane(rw::common::Ptr<rw::models::WorkCell> wc, const rw::math::Vector3D<>& n, double d, const std::string& name = "Plane", bool trimesh = false) const;
	void addBox(rw::common::Ptr<rw::models::WorkCell> wc, double x, double y, double z, double density, const std::string& name = "Box", bool trimesh = false) const;

private:
	ColorScheme _colors;
};
//! @}
} /* namespace test */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TEST_WORKCELLBUILDER_HPP_ */
