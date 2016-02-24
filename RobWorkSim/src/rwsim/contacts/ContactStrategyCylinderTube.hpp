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

#ifndef RWSIM_CONTACTS_CONTACTSTRATEGYCYLINDERTUBE_HPP_
#define RWSIM_CONTACTS_CONTACTSTRATEGYCYLINDERTUBE_HPP_

/**
 * @file ContactStrategyCylinderTube.hpp
 *
 * \copydoc rwsim::contacts::ContactStrategyCylinderTube
 */

#include "ContactModelGeometry.hpp"
#include <rwsim/contacts/ContactStrategyGeometry.hpp>

// Forward declarations
namespace rw { namespace geometry { class Cylinder; } }
namespace rw { namespace geometry { class Tube; } }
namespace rwsim { namespace contacts { class ContactStrategyTracking; } }
namespace rwsim { namespace contacts { template <class A, class B> class ContactModelGeometry; } }

namespace rwsim {
namespace contacts {
//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief Detection of contacts between cylinders and tubes - often used for Peg in Hole problems.
 */
class ContactStrategyCylinderTube: public rwsim::contacts::ContactStrategyGeometry<rw::geometry::Cylinder*, rw::geometry::Tube*> {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<ContactStrategyCylinderTube> Ptr;

	//! @brief Create new strategy.
	ContactStrategyCylinderTube();

	//! @brief Destructor
	virtual ~ContactStrategyCylinderTube();

	//! @copydoc rwsim::contacts::ContactStrategy::match
	virtual bool match(rw::common::Ptr<const rw::geometry::GeometryData> geoA, rw::common::Ptr<const rw::geometry::GeometryData> geoB);

	//! @copydoc rwsim::contacts::ContactStrategy::findContacts(rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,ContactStrategyData& data,ContactStrategyTracking& tracking,rwsim::log::SimulatorLogScope* log)
	virtual std::vector<rwsim::contacts::Contact> findContacts(
			rw::proximity::ProximityModel::Ptr a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
			const rw::math::Transform3D<>& wTb,
			ContactStrategyData& data,
			ContactStrategyTracking& tracking,
			rwsim::log::SimulatorLogScope* log = NULL) const;

	//! @copydoc rwsim::contacts::ContactStrategy::updateContacts(rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,ContactStrategyData& data,ContactStrategyTracking& tracking,rwsim::log::SimulatorLogScope* log)
	virtual std::vector<Contact> updateContacts(
			rw::proximity::ProximityModel::Ptr a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
			const rw::math::Transform3D<>& wTb,
			ContactStrategyData& data,
			ContactStrategyTracking& tracking,
			rwsim::log::SimulatorLogScope* log = NULL) const;

	//! @copydoc rwsim::contacts::ContactStrategy::getName
	virtual std::string getName();

	/**
	 * @brief The current distance threshold used by the strategy.
	 *
	 * The default threshold is 0.5 mm.
	 * This can be changed by setting the ContactStrategyPiHThreshold property in the PropertyMap.
	 * If this is not set, the MaxSepDistance property can also be used.
	 *
	 * @return the threshold (positive).
	 */
	virtual double getThreshold() const;

	/**
	 * @brief The rounding radius used by the strategy to round off sharp edges and corners.
	 *
	 * @return the rounding radius used.
	 */
	virtual double getRoundingRadius() const;

	/**
	 * @brief Set the rounding radius used by the strategy to round off sharp edges and corners.
	 *
	 * @param radius [in] the rounding radius to use.
	 */
	virtual void setRoundingRadius(double radius);

	static std::vector<rw::math::Vector3D<float> > getProjectedPoints(rw::common::Ptr<rw::geometry::Cylinder> peg, rw::math::Transform3D<> pegPose, rw::common::Ptr<rw::geometry::Tube> tube, rw::math::Transform3D<> holePose);
	static std::vector<rw::math::Vector3D<float> > getIntersectionPoints(rw::common::Ptr<rw::geometry::Cylinder> peg, rw::math::Transform3D<> pegPose, rw::common::Ptr<rw::geometry::Tube> tube, rw::math::Transform3D<> holePose);
	static std::vector<rw::math::Vector3D<float> > getHolePoints(rw::common::Ptr<rw::geometry::Tube> tube, rw::math::Transform3D<> holePose);

private:
	struct Characteristics;

	typedef enum CylinderRegion {
		SURFACE,
		TOPPLANE,
		BOTTOMPLANE,
		CYLTOPEDGE,
		CYLBOTTOMEDGE,
		CYLALL
	} CylinderRegion;

	typedef enum TubeRegion {
		INNERSURFACE,
		OUTERSURFACE,
		TUBETOPEDGE,
		TUBEBOTTOMEDGE,
		TUBEALL
	} TubeRegion;

	static std::string cylinderRegionToString(CylinderRegion region);
	static std::string tubeRegionToString(TubeRegion region);

	struct TrackInfo;
	class TrackingCylinderTube;

	struct MetaContact {
		rwsim::contacts::Contact contact;
		CylinderRegion cylinderRegion;
		TubeRegion tubeRegion;
		double caseInfo;
	};

	static Characteristics getCharacteristics(rw::common::Ptr<rw::geometry::Cylinder> peg, rw::math::Transform3D<> pegPose, rw::common::Ptr<rw::geometry::Tube> tube, rw::math::Transform3D<> holePose);

	static const double NO_THRESHOLD;

	//static bool cylSurfaceTubeSurface(const Characteristics& c, std::vector<MetaContact> &mcontacts, double threshold, CylinderRegion cylRegion = CYLALL);
	static bool cylSurfaceTubeEdge(const Characteristics& c, std::vector<MetaContact> &mcontacts, double threshold, TubeRegion tubeRegion = TUBEALL);
	static bool cylPlaneTubeEdge(const Characteristics& c, std::vector<MetaContact> &mcontacts, double threshold = -1, CylinderRegion cylRegion = CYLALL, TubeRegion tubeRegion = TUBEALL);
	static bool cylEdgeTubeSurface(const Characteristics& c, std::vector<MetaContact> &mcontacts, double threshold, CylinderRegion cylRegion = CYLALL, TubeRegion tubeRegion = TUBEALL);
	static bool cylEdgeTubeEdgeMakeContact(MetaContact& mcontact, const Characteristics& c, bool cylAtTop, bool tubeAtTop, const rw::math::Vector3D<>& cylPoint, const rw::math::Vector3D<>& tubePoint, double threshold);
	static bool cylEdgeTubeEdge(const Characteristics& c, std::vector<MetaContact> &mcontacts, double threshold = -1, CylinderRegion cylRegion = CYLALL, TubeRegion tubeRegion = TUBEALL);
/*
	static std::vector<rwsim::contacts::Contact> cylSurfaceTubeSurface(rw::geometry::Cylinder::Ptr peg, rw::math::Transform3D<> pegPose, rw::geometry::Tube::Ptr tube, rw::math::Transform3D<> holePose, double threshold, Surface tubeSurface);
	-static std::vector<rwsim::contacts::Contact> cylSurfaceTubeEdge(rw::geometry::Cylinder::Ptr peg, rw::math::Transform3D<> pegPose, rw::geometry::Tube::Ptr tube, rw::math::Transform3D<> holePose, double threshold, End tubeEnd);

	static std::vector<rwsim::contacts::Contact> cylPlaneTubeSurface(rw::geometry::Cylinder::Ptr peg, rw::math::Transform3D<> pegPose, rw::geometry::Tube::Ptr tube, rw::math::Transform3D<> holePose, double threshold, End cylinderEnd, Surface tubeSurface);
	-static std::vector<rwsim::contacts::Contact> cylPlaneTubeEdge(rw::geometry::Cylinder::Ptr peg, rw::math::Transform3D<> pegPose, rw::geometry::Tube::Ptr tube, rw::math::Transform3D<> holePose, double threshold, End cylinderEnd, End tubeEnd);

	-static std::vector<rwsim::contacts::Contact> cylEdgeTubeSurface(rw::geometry::Cylinder::Ptr peg, const rw::math::Transform3D<>& pegPose, rw::geometry::Tube::Ptr tube, const rw::math::Transform3D<>& holePose, double threshold, End cylinderEnd, Surface tubeSurface);
	-static std::vector<rwsim::contacts::Contact> cylEdgeTubeEdge(rw::geometry::Cylinder::Ptr peg, rw::math::Transform3D<> pegPose, rw::geometry::Tube::Ptr tube, rw::math::Transform3D<> holePose, double threshold, End cylinderEnd, End tubeEnd);
*/
	static std::vector<MetaContact> findContacts(const Characteristics& characteristics, double threshold);
	static std::vector<MetaContact> updateContacts(const Characteristics& characteristics, std::vector<TrackInfo>& tinfos, bool findNew = false, rwsim::log::SimulatorLogScope* log = NULL);
	static std::vector<rwsim::contacts::Contact> findContactsIntersection(rw::common::Ptr<rw::geometry::Cylinder> peg, rw::math::Transform3D<> pegPose, rw::common::Ptr<rw::geometry::Tube> tube, rw::math::Transform3D<> holePose, double threshold);

	double _rounding;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_CONTACTSTRATEGYCYLINDERTUBE_HPP_ */
