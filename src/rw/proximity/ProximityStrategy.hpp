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


#ifndef RW_PROXIMITY_PROXIMITYSTRATEGY_HPP
#define RW_PROXIMITY_PROXIMITYSTRATEGY_HPP

/**
 * @file rw/proximity/ProximityStrategy.hpp
 */

#include <string>

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/geometry/Face.hpp>

namespace rw { namespace proximity {

    /** @addtogroup proximity */
    /*@{*/

    /**
     * @brief The ProximityStrategy interface is a clean interface
     * for defining methods that are common for different proximity
     * strategy classes
     */
    class ProximityStrategy {
    public:
        /**
         * @brief Destructor.
         */
        virtual ~ProximityStrategy();

        /**
         * @brief Adds a Proximity model of a frame to this strategy.'
         *
         * The Proximity model is the one specified in the frames property
         *
         * @param frame [in] the frame on which the Proximity model is to be
         * created.
         *
         * @return true if a Proximity model was succesfully created and linked
         * with the frame; false otherwise.
         */
        virtual bool addModel(const kinematics::Frame* frame) = 0;

        /**
         * @brief Adds a Proximity model to a frame
         *
         * The Proximity model is constructed from the list of faces
         *
         * @param frame [in] the frame to which the Proximity model should associate
         * @param faces [in] list of faces from which to construct the Proximity model
         * @return true if a Proximity model was succesfully created and linked
         * with the frame; false otherwise.
         */
        virtual bool addModel(
            const rw::kinematics::Frame* frame,
            const std::vector<rw::geometry::Face<float> >& faces) = 0;

        /**
         * @brief Tells whether the frame has a proximity model in the strategy
         *
         * To have a proximity model does not means that it is loaded. If a \b GeoID string from
         * which a model can be loaded it returns true as well
         *
         * @param frame [in] the frame to check for
         * @return true if a model exists or can be created
         */
        virtual bool hasModel(const rw::kinematics::Frame* frame) = 0;

        /**
         * @brief Clears any stored model information
         */
        virtual void clear() = 0;

        /**
           @brief Clear (remove all) model information for frame \b frame.
         */
        virtual void clearFrame(const rw::kinematics::Frame* frame) = 0;

    private:
        ProximityStrategy(const ProximityStrategy&);
        ProximityStrategy& operator=(const ProximityStrategy&);

    protected:
        /**
         * @brief Creates object
         */
        ProximityStrategy();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
