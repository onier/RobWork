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

#ifndef RWSIM_CONTACTS_TUBEPLANESTRATEGY_HPP_
#define RWSIM_CONTACTS_TUBEPLANESTRATEGY_HPP_

/**
 * @file TubePlaneStrategy.hpp
 *
 * \copydoc rwsim::contacts::TubePlaneStrategy
 */

#include "ContactStrategyGeometry.hpp"

namespace rw { namespace geometry { class Tube; } }
namespace rw { namespace geometry { class Plane; } }

namespace rwsim {
namespace contacts {
//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief Detection of contacts between a tube and a plane.
 */
class TubePlaneStrategy: public ContactStrategyGeometry<rw::geometry::Tube*, rw::geometry::Plane*> {
public:
	//! @brief Create new strategy.
	TubePlaneStrategy();

	//! @brief Destructor
	virtual ~TubePlaneStrategy();

	//! @copydoc rwsim::contacts::ContactStrategy::match
	virtual bool match(rw::common::Ptr<const rw::geometry::GeometryData> geoA, rw::common::Ptr<const rw::geometry::GeometryData> geoB);

	//! @copydoc rwsim::contacts::ContactStrategy::findContacts(rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,ContactStrategyData&,ContactStrategyTracking&,rwsim::log::SimulatorLogScope*) const
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
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_TUBEPLANESTRATEGY_HPP_ */
