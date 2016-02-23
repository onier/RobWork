/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIM_CONTACTS_CONTACTSTRATEGYCYLINDERPLANE_HPP_
#define RWSIM_CONTACTS_CONTACTSTRATEGYCYLINDERPLANE_HPP_

#include <rwsim/contacts/ContactStrategyGeometry.hpp>

// Forward declarations
namespace rw { namespace geometry { class Cylinder; } }
namespace rw { namespace geometry { class Plane; } }
namespace rwsim { namespace contacts { class ContactStrategyTracking; } }
namespace rwsim { namespace contacts { template <class A, class B> class ContactModelGeometry; } }

/**
 * @file ContactStrategyCylinderPlane.hpp
 *
 * \copydoc rwsim::contacts::ContactStrategyCylinderPlane
 */

namespace rwsim {
namespace contacts {
//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief Detection of contacts between a cylinder and a plane.
 */
class ContactStrategyCylinderPlane: public rwsim::contacts::ContactStrategyGeometry<rw::geometry::Cylinder*, rw::geometry::Plane*> {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<ContactStrategyCylinderPlane> Ptr;

	//! @brief Create new strategy.
	ContactStrategyCylinderPlane();

	//! @brief Destructor
	virtual ~ContactStrategyCylinderPlane();

	//! @copydoc rwsim::contacts::ContactStrategy::match
	virtual bool match(rw::common::Ptr<const rw::geometry::GeometryData> geoA, rw::common::Ptr<const rw::geometry::GeometryData> geoB);

	//! @copydoc rwsim::contacts::ContactStrategy::findContacts(rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,ContactStrategyData& data,ContactStrategyTracking& tracking,rwsim::log::SimulatorLogScope*)
	virtual std::vector<rwsim::contacts::Contact> findContacts(
			rw::proximity::ProximityModel::Ptr a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
			const rw::math::Transform3D<>& wTb,
			rwsim::contacts::ContactStrategyData& data,
			rwsim::contacts::ContactStrategyTracking& tracking,
			rwsim::log::SimulatorLogScope* log = NULL) const;

	//! @copydoc rwsim::contacts::ContactStrategy::updateContacts(rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,ContactStrategyData& data,ContactStrategyTracking& tracking,rwsim::log::SimulatorLogScope*)
	virtual std::vector<rwsim::contacts::Contact> updateContacts(
			rw::proximity::ProximityModel::Ptr a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
			const rw::math::Transform3D<>& wTb,
			rwsim::contacts::ContactStrategyData& data,
			rwsim::contacts::ContactStrategyTracking& tracking,
			rwsim::log::SimulatorLogScope* log = NULL) const;

	//! @copydoc rwsim::contacts::ContactStrategy::getName
	virtual std::string getName();

	//! @brief Location specifies where to find contacts - at the top or bottom or both.
	typedef enum Location {
		TOP,   //!< TOP
		BOTTOM,//!< BOTTOM
		BOTH   //!< BOTH
	} Location;

	/**
	 * @brief Get the nearest points at one or both ends (user should check that cylinder is not parallel to plane).
	 * @param cylinder [in] the cylinder geometry.
	 * @param cylPose [in] the pose of the cylinder relative to world.
	 * @param plane [in] the plane geometry.
	 * @param planePose [in] the pose of the plane relative to world.
	 * @param end [in] which ends to include (TOP, BOTTOM or BOTH).
	 * @return a list of contacts - either with one or two contacts according to the choice of end(s).
	 */
	static std::vector<Contact> getNearest(rw::common::Ptr<rw::geometry::Cylinder> cylinder, rw::math::Transform3D<> cylPose, rw::common::Ptr<rw::geometry::Plane> plane, rw::math::Transform3D<> planePose, Location end);

	/**
	 * @brief Sample points at one of the ends of the cylinder/tube (for when it is mostly parallel to the plane).
	 * @param cylinderOrTube [in] the cylinder or tube geometry.
	 * @param pose [in] the pose of the cylinder/tube relative to world.
	 * @param plane [in] the plane geometry.
	 * @param planePose [in] the pose of the plane relative to world.
	 * @param end [in] which ends to include (TOP or BOTTOM).
	 * @param samples [in] the number of samples (default is 3).
	 * @return a list of contacts of length similar to the number of samples chosen.
	 */
	static std::vector<Contact> sampleEnd(rw::common::Ptr<rw::geometry::GeometryData> cylinderOrTube, rw::math::Transform3D<> pose, rw::common::Ptr<rw::geometry::Plane> plane, rw::math::Transform3D<> planePose, Location end, unsigned int samples = 3);

private:
	typedef enum Case {
		NONE,
		BEND,
		BEDGE,
		SURFACE,
		TEDGE,
		TEND,
		_BEND2,
		_TEND2
	} Case;

	class CylinderData;
	struct TrackInfo;
	class TrackingCylinderPlane;

	static Case getCurrentCase(const std::vector<TrackInfo>& tinfos);

	//typedef std::pair<rwsim::contacts::Contact,Location> MetaContact;
	//static std::vector<MetaContact> findContacts(rw::common::Ptr<rw::geometry::Cylinder> cylinder, rw::math::Transform3D<> cylPose, rw::common::Ptr<rw::geometry::Plane> plane, rw::math::Transform3D<> planePose, rw::common::Ptr<const CylinderData> cylData);
	//static std::vector<MetaContact> updateContacts(rw::common::Ptr<rw::geometry::Cylinder> cylinder, rw::math::Transform3D<> cylPose, rw::common::Ptr<rw::geometry::Plane> plane, rw::math::Transform3D<> planePose, rw::common::Ptr<const CylinderData> cylData, std::vector<TrackInfo>& tinfos, bool findNew = false);

	static std::vector<Contact> updateContactsCase(rw::common::Ptr<rw::geometry::Cylinder> cylinder, rw::math::Transform3D<> cylPose, rw::common::Ptr<rw::geometry::Plane> plane, rw::math::Transform3D<> planePose, rw::common::Ptr<const CylinderData> cylData, std::vector<TrackInfo>& tinfos, bool findNew = false);
	static std::vector<Contact> updateContactsTopEdge(rw::common::Ptr<rw::geometry::Cylinder> cylinder, rw::math::Transform3D<> cylPose, rw::common::Ptr<rw::geometry::Plane> plane, rw::math::Transform3D<> planePose, rw::common::Ptr<const CylinderData> cylData, std::vector<TrackInfo>& tinfos, bool findNew = false);
	static std::vector<Contact> updateContactsBottomEdge(rw::common::Ptr<rw::geometry::Cylinder> cylinder, rw::math::Transform3D<> cylPose, rw::common::Ptr<rw::geometry::Plane> plane, rw::math::Transform3D<> planePose, rw::common::Ptr<const CylinderData> cylData, std::vector<TrackInfo>& tinfos, bool findNew = false);
	static std::vector<Contact> updateContactsSurface(rw::common::Ptr<rw::geometry::Cylinder> cylinder, rw::math::Transform3D<> cylPose, rw::common::Ptr<rw::geometry::Plane> plane, rw::math::Transform3D<> planePose, rw::common::Ptr<const CylinderData> cylData, std::vector<TrackInfo>& tinfos, bool findNew = false);
	static std::vector<Contact> updateContactsBottomEnd(Case curCase, rw::common::Ptr<rw::geometry::Cylinder> cylinder, rw::math::Transform3D<> cylPose, rw::common::Ptr<rw::geometry::Plane> plane, rw::math::Transform3D<> planePose, rw::common::Ptr<const CylinderData> cylData, std::vector<TrackInfo>& tinfos, bool findNew = false);

	static std::vector<Contact> getNearest(rw::common::Ptr<rw::geometry::Cylinder> cylinder, rw::math::Transform3D<> cylPose, rw::common::Ptr<rw::geometry::Plane> plane, rw::math::Transform3D<> planePose, rw::common::Ptr<const CylinderData> cylData, Location end);
	static std::vector<Contact> sampleEnd(rw::common::Ptr<rw::geometry::GeometryData> cylinderOrTube, rw::math::Transform3D<> cylPose, rw::common::Ptr<rw::geometry::Plane> plane, rw::math::Transform3D<> planePose, rw::common::Ptr<const CylinderData> cylData, Location end, unsigned int samples = 3);
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* CONTACTSTRATEGYCYLINDERPLANE_HPP_ */
