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


#ifndef RW_COLLISION_COLLISIONDETECTOR_HPP
#define RW_COLLISION_COLLISIONDETECTOR_HPP

/**
 * @file rw/proximity/CollisionDetector.hpp
 *
 * @brief Class rw::proximity::CollisionDetector
 */

#include "Proximity.hpp"
#include "CollisionSetup.hpp"
#include "CollisionStrategy.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

#include <vector>

namespace rw {
    namespace kinematics {
        class Frame;
    }
}

namespace rw {
    namespace proximity {

    /** @addtogroup proximity */
    /*@{*/

    class CollisionDetector;

    //! A pointer to a CollisionDetector.
    typedef rw::common::Ptr<CollisionDetector> CollisionDetectorPtr;

    /**
     @brief The CollisionDetector implements an efficient way of checking a
     complete frame tree for collisions.

     It contain a set of pairs of frames that are not to be checked against
     each other. The collision detector does not dictate a specific detection
     strategy or algorithm, instead it relies on the CollisionStrategy interface for
     the actual collision checking between two frames.

     The CollisionDetector supports switching between multiple strategies.
     */
    class CollisionDetector
    {
    public:
        /**
         @brief Collision detector for a workcell.

         The default collision setup stored in the workcell is used.

         @param workcell [in] the workcell.

         @param strategy [in] the collision checker strategy to use.
         */
        CollisionDetector(rw::models::WorkCellPtr workcell,
                          CollisionStrategyPtr strategy);

        /**
         @brief Collision detector for a workcell.

         Collision checking is done for the provided collision setup alone.

         @param workcell [in] the workcell.

         @param strategy [in] the collision checker strategy to use.

         @param setup [in] the setup for the collision checking.
         */
        CollisionDetector(rw::models::WorkCellPtr workcell,
                          CollisionStrategyPtr strategy,
                          const CollisionSetup& setup);

        /**
         @brief Check the workcell for collisions.

         @param state [in] The state for which to check for collisions.

         @param result [out] If non-NULL, the pairs of colliding frames are
         inserted in \b result.

         @param stopAtFirstContact [in] If \b result is non-NULL and \b
         stopAtFirstContact is true, then only the first colliding pair is
         inserted in \b result. By default all colliding pairs are inserted.

         @return true if a collision is detected; false otherwise.
         */
        bool inCollision(const kinematics::State& state,
						 kinematics::FramePairSet* result = 0,
                         bool stopAtFirstContact = false) const;

        /**
         @brief Set the primitive collision strategy to \b strategy.

         \b strategy must be non-NULL.

         @param strategy [in] - the primitive collision checker to use.
         */
        void setCollisionStrategy(CollisionStrategyPtr strategy);

        /**
         @brief The collision strategy of the collision checker.
         */
        CollisionStrategy& getCollisionStrategy() const
        {
            return *_strategy;
        }

        /**
         @brief The collision strategy of the collision checker.
         */
        CollisionStrategyPtr getCollisionStrategyPtr() const
        {
            return _strategy;
        }

        /**
         * @brief Returns the frame pairs which will be checked for collision
         */
        const kinematics::FramePairSet& getFramePairSet() const
        {
            return _collisionPairs;
        }

        /**
         * @brief updates the collision detector such that changes to frames,
         * collision strategies and such is updated.
         *
         * @example a new model has been added to the collision strategy. If
         * the frame did not have a model prior to adding, it is necesary to call
         * update on the collision detector.
         */
        //void update();

        // Constructor functions.

        /**
         @brief Collision detector for a workcell and collision setup.

         The default collision setup stored in the workcell is used.

         @param workcell [in] the workcell.

         @param strategy [in] the collision checker strategy to use.
         */
        static CollisionDetectorPtr make(rw::models::WorkCellPtr workcell,
                                         CollisionStrategyPtr strategy);

        /**
         @brief Collision detector for a workcell.

         Collision checking is done for the provided collision setup alone.

         @param workcell [in] the workcell.

         @param strategy [in] the collision checker strategy to use.

         @param setup [in] the setup for the collision checking.
         */
        static CollisionDetectorPtr make(rw::models::WorkCellPtr workcell,
                                         CollisionStrategyPtr strategy,
                                         const CollisionSetup& setup);

        /**
         @brief Collision detector for a set of pairs of frames.

         The collision detector checks if any of the pairs of frames are in
         collision.

         \b strategy must be non-NULL.

         @param strategy [in] Collision checker for a frame pair.
         @param pairs [in] Pairs of frames.
         */
        static CollisionDetectorPtr make(CollisionStrategyPtr strategy,
                                         const kinematics::FramePairSet& pairs);

        /**
         @brief Collision detector for a device.

         This collision detector assumes that all frames of the workcell
         (including the DAFs) are fixed, except for the frames that can be
         controlled by \b device.

         The collision also assumes that frame pairs that are not in the set
         of pairs stored in \b detector need not be checked.

         The collision strategy of \b detector is reused for the new collision
         detector.
         */
        static CollisionDetectorPtr
                make(const CollisionDetector& detector,
                     const rw::models::Device& device,
                     const rw::kinematics::State& state);

        /*
         @brief A pair (\b detectorStatic, \b detectorDynamic) for a sequence
         of dynamic obstacles \b obstacleDevices changing the shape of the
         configuration space.

         \b detectorStatic is found by removing all geometries controlled by
         \b obstacleDevices for the given state with DAFs are treated as
         fixed frames.

         \b detectorDynamic is found by including only pairs of geometries
         relating a geometry of \b obstacleDevices to a geometry of \b
         controlledDevices for the given state with DAFs are treated as fixed
         frames.

         The set of collision pairs of \b detector is used as the total set of
         pairs from which pairs are excluded to yield \b detectorStatic and \b
         detectorDynamic.
         */
        static
        std::pair<CollisionDetectorPtr, CollisionDetectorPtr>
                makeStaticDynamic(
                                  const CollisionDetector& detector,
                                  const std::vector<rw::models::DevicePtr>& obstacleDevices,
                                  const std::vector<rw::models::DevicePtr>& controlledDevices,
                                  const rw::kinematics::State& state);



    private:
        CollisionStrategyPtr _strategy;
        kinematics::FramePairSet _collisionPairs;

    private:
        CollisionDetector(CollisionStrategyPtr strategy,
                          const kinematics::FramePairSet& pairs);

        CollisionDetector(const CollisionDetector&);
        CollisionDetector& operator=(const CollisionDetector&);

    };

/*@}*/
}
} // end namespaces

#endif // end include guard
