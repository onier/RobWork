/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_PROXIMITY_PROXIMITYDATA_HPP_
#define RW_PROXIMITY_PROXIMITYDATA_HPP_

/**
 * @file rw/proximity/ProximityData.hpp
 */

#include "CollisionDetector.hpp"

namespace rw {
namespace proximity {
    class ProximityCache;

    //! @addtogroup proximity
    //! @{

    /**
     * @brief Holds settings and cached data for collision detectors.
     *
     * The cache makes it possible for some algorithms to perform faster
     * detections.
     */
    class ProximityData {
        public:
            /**
             * @brief Default constructor.
             *
             * By default, the collision detector returns on first contact
             * with no detailed information about the collision.
             *
             * Use setCollisionQueryType to change this behaviour.
             */
            ProximityData():
                _colQueryType(CollisionDetector::FirstContactNoInfo)
            {
            }

            /**
             * @brief Set the type of collision query.
             *
             * The detection can perform faster if it is allowed to return
             * after detecting the first collision. Alternatively, it is
             * possible to detect all collisions if required.
             *
             * @param qtype [in] the query type.
             * @see CollisionDetector::QueryType
             */
            void setCollisionQueryType(CollisionDetector::QueryType qtype) {
                _colQueryType = qtype;
            }

            /**
             * @brief Get the collision query type.
             * @return the query type.
             * @see CollisionDetector::QueryType
             */
            CollisionDetector::QueryType getCollisionQueryType() const {
                return _colQueryType;
            }

            /**
             * @brief Detailed information about the collision.
             * @note This data is only available for some collision query types.
             * @see CollisionDetector::QueryResult
             */
            CollisionDetector::QueryResult _collisionData;

            /**
             * @brief Cached data used by the collision detector to speed up
             * consecutive queries.
             */
            rw::common::Ptr<ProximityCache> _cache;

        private:
            CollisionDetector::QueryType _colQueryType;
	};

	//! @}
}
}

#endif /* PROXIMITYDATA_HPP_ */
