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

#ifndef RWSIM_CONTACTS_BALLBALLSTRATEGY_HPP_
#define RWSIM_CONTACTS_BALLBALLSTRATEGY_HPP_

/**
 * @file BallBallStrategy.hpp
 *
 * \copydoc rwsim::contacts::BallBallStrategy
 */

#include "ContactStrategy.hpp"
#include "ContactStrategyGeometry.hpp"

namespace rw { namespace geometry { class Sphere; } }

namespace rwsim {
namespace contacts {
//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief Detection of contacts between balls. Each model can consist of multiple balls.
 */
class BallBallStrategy: public ContactStrategyGeometry<rw::geometry::Sphere*, rw::geometry::Sphere*> {
public:
	//! @brief Create new strategy.
	BallBallStrategy();

	//! @brief Destructor
	virtual ~BallBallStrategy();

	//! @copydoc rwsim::contacts::ContactStrategy::match
	virtual bool match(rw::common::Ptr<const rw::geometry::GeometryData> geoA, rw::common::Ptr<const rw::geometry::GeometryData> geoB);

	//! @copydoc rwsim::contacts::ContactStrategy::findContacts(rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,ContactStrategyData&,ContactStrategyTracking&,rwsim::log::SimulatorLogScope* log) const
	virtual std::vector<Contact> findContacts(
			rw::proximity::ProximityModel::Ptr a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
			const rw::math::Transform3D<>& wTb,
			ContactStrategyData& data,
			ContactStrategyTracking& tracking,
			rwsim::log::SimulatorLogScope* log = NULL) const;

	//! @copydoc rwsim::contacts::ContactStrategy::updateContacts
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
	 * @brief Determine the contact between two spheres.
	 * @param c [in/out] the contact to add information to.
	 * @param a [in] the first sphere geometry.
	 * @param wPa [in] the location of the center of the first sphere.
	 * @param b [in] the second sphere geometry.
	 * @param wPb [in] the location of the center of the second sphere.
	 * @param distCheck [in] (optional) only detect contact if distance is low enough (default is true).
	 * @return true if contact found, false otherwise.
	 */
	virtual bool findContact(Contact &c,
			const rw::geometry::Sphere* a,	const rw::math::Vector3D<>& wPa,
			const rw::geometry::Sphere* b,	const rw::math::Vector3D<>& wPb, bool distCheck = true) const;

	//! @copydoc rwsim::contacts::ContactStrategy::destroyModel
    virtual void destroyModel(rw::proximity::ProximityModel* model);

	//! @copydoc rwsim::contacts::ContactStrategy::addGeometry(rw::proximity::ProximityModel*,const rw::geometry::Geometry&)
	virtual bool addGeometry(rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom);

	//! @copydoc rwsim::contacts::ContactStrategy::addGeometry(rw::proximity::ProximityModel*,rw::common::Ptr<rw::geometry::Geometry>,bool)
    virtual bool addGeometry(rw::proximity::ProximityModel* model, rw::common::Ptr<rw::geometry::Geometry> geom, bool forceCopy=false);

	//! @copydoc rwsim::contacts::ContactStrategy::removeGeometry
    virtual bool removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId);

	//! @copydoc rwsim::contacts::ContactStrategy::getGeometryIDs
    virtual std::vector<std::string> getGeometryIDs(rw::proximity::ProximityModel* model);

	//! @copydoc rwsim::contacts::ContactStrategy::clear
    virtual void clear();

private:
	class BallTracking;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_BALLBALLSTRATEGY_HPP_ */
