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

#ifndef RWSIMLIBS_RWPE_RWPEUTIL_HPP_
#define RWSIMLIBS_RWPE_RWPEUTIL_HPP_

/**
 * @file RWPEUtil.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEUtil
 */

#include <vector>
#include <set>
#include <list>

#include <rw/math/Vector3D.hpp>
#include <rwsim/contacts/ContactStrategyTracking.hpp>

// Forward declarations
namespace rw { namespace kinematics { class Frame; } };
namespace rw { namespace kinematics { class State; } };
namespace rwlibs { namespace simulation { class SimulatedSensor; } };
namespace rwsim { namespace contacts { class Contact; } };
namespace rwsim { namespace contacts { class ContactDetectorTracking; } };

namespace rwsimlibs {
namespace rwpe {

// Forward declarations
class RWPEBodyConstraintGraph;
class RWPEIslandState;
class RWPEContact;

//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief Utility functions for the RWPE physics engine.
 *
 * Most functions are related to contact tracking.
 * Contacts can in general be in two states, either marked or known.
 *
 * Marked contacts are contacts that belongs to a specific class of contacts.
 * The marked contacts can not need be tracked individually, but only as a group.
 *
 * All new contacts detected by the ContactDetector will be marked MARK_RAW.
 *
 * A predefined mark, MARK_NEW, is defined for contacts that are candidates for
 * being treated as bouncing. This is used mainly during rollback to limit the effort
 * used on contact detection.
 *
 * Finally known contacts are contacts that has a RWPEContact attached as user data.
 * This allows tracking existing known contacts to update the contact positions.
 *
 * Besides the contact tracking functions, there are some helper functions defined
 * with the purpose of making the main simulation loop easier to read.
 */
class RWPEUtil {
public:
	//! @brief Tracking structure that utilizes the features of the contact detector with tracking.
	struct RWPEUserData: public rwsim::contacts::ContactStrategyTracking::UserData {
		/**
		 * @brief Construct new data for the given RWPEContact.
		 * @param contact [in] the contact to assign to this user-data structure.
		 */
		RWPEUserData(const RWPEContact* contact): contact(contact) {}

		//! @brief The RWPEContact assigned to the given contact.
		const RWPEContact* const contact;

		/**
		 * @brief Construct new data for the given contact.
		 * @param contact [in] the contact
		 * @return new data.
		 */
		static rwsim::contacts::ContactStrategyTracking::UserData::Ptr make(const RWPEContact* contact) {
			return rw::common::ownedPtr(new RWPEUserData(contact));
		}
	};

	//! @brief Predefined mark that can be used to mark new contacts.
	static const rwsim::contacts::ContactStrategyTracking::UserData::Ptr MARK_NEW;

	//! @brief Predefined mark that can is used for newly detected contacts.
	static const rwsim::contacts::ContactStrategyTracking::UserData::Ptr MARK_RAW;

	/**
	 * @brief Find the contacts with the given mark.
	 * @param input [in] the list of contacts to classify.
	 * @param tracking [in] the tracking information from the rwsim::contacts::ContactDetector.
	 * @param mark [in] the mark to search for.
	 * @return a list of indices for contacts that has the given mark.
	 */
	static std::vector<std::size_t> getMarkedContacts(const std::vector<rwsim::contacts::Contact>& input, const rwsim::contacts::ContactDetectorTracking& tracking, rwsim::contacts::ContactStrategyTracking::UserData::Ptr mark);

	/**
	 * @brief Get the contacts refered to by a list of indices.
	 * @param contacts [in] the contacts.
	 * @param ids [in] the indices.
	 * @return new list with only the contacts in the id list
	 */
	static std::vector<rwsim::contacts::Contact> getContacts(const std::vector<rwsim::contacts::Contact>& contacts, const std::vector<std::size_t>& ids);

	/**
	 * @brief Remove the new contacts that are not in penetration and leave the new penetrating contacts.
	 * @param contacts [in/out] the complete contact list.
	 * @param tracking [in/out] the tracking information.
	 * @param mark [in] only remove contacts that has this mark.
	 */
	static void removeNonPenetrating(std::vector<rwsim::contacts::Contact>& contacts, rwsim::contacts::ContactDetectorTracking& tracking, rwsim::contacts::ContactStrategyTracking::UserData::Ptr mark);

	/**
	 * @brief Remove the new contacts that are in penetration and leave the new non-penetrating contacts.
	 * @param contacts [in/out] the complete contact list.
	 * @param tracking [in/out] the tracking information.
	 * @param mark [in] only remove contacts that has this mark.
	 */
	static void removePenetrating(std::vector<rwsim::contacts::Contact>& contacts, rwsim::contacts::ContactDetectorTracking& tracking, rwsim::contacts::ContactStrategyTracking::UserData::Ptr mark);

	/**
	 * @brief Remove all known contacts, leaving only the marked ones.
	 * @param contacts [in/out] the complete contact list.
	 * @param tracking [in/out] the tracking information.
	 * @param bc [in] the body-constraint manager that has the constraints in the system.
	 * @param islandState [in] the state.
	 */
	static void removeKnown(std::vector<rwsim::contacts::Contact>& contacts, rwsim::contacts::ContactDetectorTracking& tracking, const RWPEBodyConstraintGraph* bc, const RWPEIslandState& islandState);

	/**
	 * @brief Remove all contacts with the given mark.
	 * @param contacts [in/out] the complete contact list.
	 * @param tracking [in/out] the tracking information.
	 * @param mark [in] only remove contacts that has this mark.
	 */
	static void remove(std::vector<rwsim::contacts::Contact>& contacts, rwsim::contacts::ContactDetectorTracking& tracking, rwsim::contacts::ContactStrategyTracking::UserData::Ptr mark);

	/**
	 * @brief Mark all contacts with a given mark with a different mark.
	 * @param contacts [in/out] the complete contact list.
	 * @param tracking [in/out] the tracking information.
	 * @param oldMark [in] the old mark.
	 * @param newMark [in] the new mark.
	 */
	static void mark(std::vector<rwsim::contacts::Contact>& contacts, rwsim::contacts::ContactDetectorTracking& tracking, rwsim::contacts::ContactStrategyTracking::UserData::Ptr oldMark, rwsim::contacts::ContactStrategyTracking::UserData::Ptr newMark);

	/**
	 * @brief Do integration for all bodies and update the state structures.
	 * @param dt [in] the stepsize.
	 * @param gravity [in] the gravity in world coordinates.
	 * @param discontinuity [in] integrate as first step after a discontinuity.
	 * @param bc [in] the body-constraint manager.
	 * @param islandState [in/out] the current state to update.
	 * @param rwstate [in/out] the current state to update.
	 */
	static void step(double dt, const rw::math::Vector3D<>& gravity, bool discontinuity, const RWPEBodyConstraintGraph* bc, RWPEIslandState& islandState, rw::kinematics::State& rwstate);

	/**
	 * @brief Do integration of all body positions and update the state structures.
	 * @param dt [in] the stepsize.
	 * @param gravity [in] the gravity in world coordinates.
	 * @param discontinuity [in] integrate as first step after a discontinuity.
	 * @param bc [in] the body-constraint manager.
	 * @param islandState [in/out] the current state to update.
	 * @param rwstate [in/out] the current state to update.
	 */
	static void positionUpdate(double dt, const rw::math::Vector3D<>& gravity, bool discontinuity, const RWPEBodyConstraintGraph* bc, RWPEIslandState& islandState, rw::kinematics::State& rwstate);

	/**
	 * @brief Do integration of all body velocities and update the state structures.
	 * @param dt [in] the stepsize.
	 * @param gravity [in] the gravity in world coordinates.
	 * @param discontinuity [in] integrate as first step after a discontinuity.
	 * @param bc [in] the body-constraint manager.
	 * @param islandState0 [in] the initial state.
	 * @param islandStateH [in/out] the next state, which is updated.
	 * @param rwstate [in] the next state.
	 * @param log [in] logging.
	 */
	static void velocityUpdate(double dt, const rw::math::Vector3D<>& gravity, bool discontinuity, const RWPEBodyConstraintGraph* bc, const RWPEIslandState& islandState0, RWPEIslandState& islandStateH, const rw::kinematics::State& rwstate, class RWPELogUtil* log);

	/**
	 * @brief Get the minimum distance in a list of contacts.
	 * @param contacts [in] the list of contacts.
	 * @return the minimum distance.
	 */
	static double minDistance(const std::vector<rwsim::contacts::Contact>& contacts);

	/**
	 * @brief Get the maximum distance in a list of contacts.
	 * @param contacts [in] the list of contacts.
	 * @return the maximum distance.
	 */
	static double maxDistance(const std::vector<rwsim::contacts::Contact>& contacts);

	/**
	 * @brief Remove contacts with specific mark that has a distance greater than some threshold.
	 * @param contacts [in/out] the list of contacts.
	 * @param tracking [in/out] the tracking information.
	 * @param threshold [in] the threshold.
	 * @param mark [in] only the contacts with this mark can be removed.
	 */
	static void removeContactsOutsideThreshold(std::vector<rwsim::contacts::Contact>& contacts, rwsim::contacts::ContactDetectorTracking& tracking, double threshold, rwsim::contacts::ContactStrategyTracking::UserData::Ptr mark);

	/**
	 * @brief Update the temporary contacts with new contact information.
	 * @param contacts [in] the updated contact information.
	 * @param tracking [in] the tracking information.
	 * @param bc [in] the body-constraint manager.
	 * @param islandState [in] the current state to update with new information.
	 * @param rwstate [in] the current state.
	 */
	static void updateTemporaryContacts(const std::vector<rwsim::contacts::Contact>& contacts, const rwsim::contacts::ContactDetectorTracking& tracking, const RWPEBodyConstraintGraph* bc, RWPEIslandState& islandState, const rw::kinematics::State &rwstate);

	/**
	 * @brief Update the sensors.
	 * @param sensors [in] the list of sensors to update.
	 * @param time [in] the current simulation time.
	 * @param dt [in] the timestep.
	 * @param dt_prev [in] previous timestep.
	 * @param bc [in] the body-constraint manager.
	 * @param islandState0 [in] the previous state.
	 * @param islandStateH [in] the current state to update with new information.
	 * @param rwstate [in] the current state.
	 * @param log [in] logging.
	 */
	static void updateSensors(const std::list<rw::common::Ptr<rwlibs::simulation::SimulatedSensor> >& sensors, double time, double dt, double dt_prev, const RWPEBodyConstraintGraph* bc, const RWPEIslandState& islandState0, const RWPEIslandState& islandStateH, rw::kinematics::State& rwstate, class RWPELogUtil& log);

private:
	static rwsim::contacts::ContactStrategyTracking::UserData::Ptr getNewMark();
	class DummyMark: public rwsim::contacts::ContactStrategyTracking::UserData {};
	RWPEUtil() {};
	virtual ~RWPEUtil() {};
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEUTIL_HPP_ */