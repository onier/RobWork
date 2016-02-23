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

#ifndef RWSIMLIBS_TEST_DYNAMICWORKCELLBUILDER_HPP_
#define RWSIMLIBS_TEST_DYNAMICWORKCELLBUILDER_HPP_

/**
 * @file DynamicWorkCellBuilder.hpp
 *
 * \copydoc rwsimlibs::test::DynamicWorkCellBuilder
 */

#include <rw/common/Ptr.hpp>
#include <rw/math/Vector3D.hpp>

namespace rwsim { namespace dynamics { class DynamicWorkCell; } }
namespace rwsim { namespace dynamics { struct BodyInfo; } }

namespace rwsimlibs {
namespace test {
//! @addtogroup INSERT_DOC_GROUP

//! @{
/**
 * @brief INSERT_SHORT_DESCRIPTION
 */
class DynamicWorkCellBuilder {
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

	DynamicWorkCellBuilder(const ColorScheme& colors = PaHColors());
	virtual ~DynamicWorkCellBuilder();

	void addFloor(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, const std::string& name = "Floor", bool trimesh = false) const;
	void addPlane(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, const rw::math::Vector3D<>& n, double d, const std::string& name = "Plane", bool trimesh = false) const;
	void addBall(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, double radius, double density, const std::string& name = "Ball", const std::string& parent = "WORLD") const;
	void addBallFixed(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, double radius, const std::string& name = "Ball", const std::string& parent = "WORLD") const;
	void addCylinder(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, double radius, double height, double density, const std::string& name = "Cylinder", const std::string& parent = "WORLD", bool trimesh = false) const;
	void addTube(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, double radius, double thickness, double height, double density, const std::string& name = "Tube", bool trimesh = false) const;
	void addBox(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, double x, double y, double z, double density, const std::string& name = "Box", bool trimesh = false) const;
	void addBoxKin(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, double x, double y, double z, double density, const std::string& name = "Box", bool trimesh = false) const;
	static rwsim::dynamics::BodyInfo defaultInfo();
	static void defaultInfo(rwsim::dynamics::BodyInfo& info);
	static void ballInfo(rwsim::dynamics::BodyInfo& info, double radius, double density);
	static void cylinderInfo(rwsim::dynamics::BodyInfo& info, double radius, double height, double density);
	static void tubeInfo(rwsim::dynamics::BodyInfo& info, double radius, double thickness, double height, double density);
	static void boxInfo(rwsim::dynamics::BodyInfo& info, double x, double y, double z, double density);
	static void addMaterialData(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, double friction = 0, double restitution = 0);

	static void contactsExclude(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, const std::string& bodyA, const std::string& bodyB);

private:
	ColorScheme _colors;
};
//! @}
} /* namespace test */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TEST_DYNAMICWORKCELLBUILDER_HPP_ */
