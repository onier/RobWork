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


#ifndef RWLIBS_PROXIMITYSTRATEGIES_PROXIMITYSTRATEGYPQP_HPP
#define RWLIBS_PROXIMITYSTRATEGIES_PROXIMITYSTRATEGYPQP_HPP

/**
 * @file ProximityStrategyPQP.hpp
 */

#include <map>
#include <vector>
#include <list>

#include <boost/shared_ptr.hpp>

#include <rw/common/Cache.hpp>

#include <rw/kinematics/Frame.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/CollisionToleranceStrategy.hpp>
#include <rw/proximity/DistanceStrategy.hpp>
#include <rw/proximity/DistanceToleranceStrategy.hpp>

#include <PQP/PQP.h>

namespace PQP { class PQP_Model; }

namespace rwlibs { namespace proximitystrategies {
    /** @addtogroup proximitystrategies */
    /*@{*/

    /**
     * @brief This is a strategy wrapper for the distance library
     * PQP (Proximity Query Package).
     *
     * PQP use Oriented Bounding Boxes (OBB) and hierarchical bounding trees for
     * fast distance calculation.
     *
     * For further information check out http://www.cs.unc.edu/~geom/SSV/
     */
    class ProximityStrategyPQP :
        public rw::proximity::CollisionStrategy,
        public rw::proximity::CollisionToleranceStrategy,
        public rw::proximity::DistanceStrategy,
        public rw::proximity::DistanceToleranceStrategy
    {
    public:
        typedef rw::common::Ptr<PQP::PQP_Model> PQPModelPtr;
        typedef std::pair<rw::math::Transform3D<>, PQPModelPtr> RWPQPModel;
        typedef std::vector<RWPQPModel> RWPQPModelList;
        typedef std::pair<RWPQPModel, RWPQPModel> RWPQPModelPair;

        struct PQPProximityModel : public rw::proximity::ProximityModel {
            PQPProximityModel(ProximityStrategy *owner):
                ProximityModel(owner)
            {
            }
            RWPQPModelList models;
        };

    private:
        bool _firstContact;
        rw::common::Cache<std::string, PQP::PQP_Model> _modelCache;

    public:
        /**
         * @brief Constructor
         */
        ProximityStrategyPQP();

        //// interface of ProximityStrategy

        /**
         * @copydoc rw::proximity::ProximityStrategy::createModel
         */
        virtual rw::proximity::ProximityModelPtr createModel();

        /**
         * @copydoc rw::proximity::ProximityStrategy::destroyModel
         */
        void destroyModel(rw::proximity::ProximityModel* model);

        /**
         * @copydoc rw::proximity::ProximityStrategy::addGeometry
         */
        bool addGeometry(rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom);

        /**
         * @copydoc rw::proximity::ProximityStrategy::removeGeometry
         */
        bool removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId);

        /**
         * @copydoc rw::proximity::ProximityStrategy::getGeometryIDs
         */
        std::vector<std::string> getGeometryIDs(rw::proximity::ProximityModel* model);

        /**
         * @copydoc rw::proximity::CollisionStrategy::setFirstContact
         */
        void setFirstContact(bool b);

        /**
         * @copydoc rw::proximity::CollisionStrategy::inCollision
         */
        bool collides(
            rw::proximity::ProximityModelPtr a,
            const rw::math::Transform3D<>& wTa,
            rw::proximity::ProximityModelPtr b,
            const rw::math::Transform3D<>& wTb);

        /**
         * @copydoc rw::proximity::CollisionToleranceStrategy::inCollision
         */
        bool collides(
            rw::proximity::ProximityModelPtr a,
            const rw::math::Transform3D<>& wTa,
            rw::proximity::ProximityModelPtr b,
            const rw::math::Transform3D<>& wTb,
            double tolerance);

        /**
         * @copydoc rw::proximity::DistanceStrategy::distance
         */
        bool calcDistance(
            rw::proximity::DistanceResult &result,
            rw::proximity::ProximityModelPtr a,
            const rw::math::Transform3D<>& wTa,
            rw::proximity::ProximityModelPtr b,
            const rw::math::Transform3D<>& wTb,
            double rel_err = 0.0,
            double abs_err = 0.0);

        /**
         * @copydoc rw::proximity::DistanceToleranceStrategy::getDistances
         */
        bool calcDistances(
            rw::proximity::MultiDistanceResult &result,
            rw::proximity::ProximityModelPtr a,
            const rw::math::Transform3D<>& wTa,
            rw::proximity::ProximityModelPtr b,
            const rw::math::Transform3D<>& wTb,
            double tolerance,
            double rel_err = 0.0,
            double abs_err = 0.0);

        /**
         *  @copydoc rw::proximity::ProximityStrategy::clear
         */
        void clear();

        /**
           @brief A PQP based collision strategy.
        */
        static rw::proximity::CollisionStrategyPtr make();

        /**
         * @brief returns the number of bounding volume tests performed
         * since the last call to clearStats
         */
        int getNrOfBVTests(){return _numBVTests;};

        /**
         * @brief returns the number of ptriangle tests performed
         * since the last call to clearStats
         */
        int getNrOfTriTests(){return _numTriTests;};

        /**
         * @brief clears the bounding volume and triangle test counters.
         */
        void clearStats(){ _numBVTests = 0; _numTriTests = 0;};
    private:
    	int _numBVTests,_numTriTests;

    	std::vector<RWPQPModel> _allmodels;
    	std::map<std::string, std::vector<int> > _geoIdToModelIdx;

//    	PQP::PQP_CollideResult _result;
//    	PQP::PQP_DistanceResult _distResult;
    };

}} // end namespaces

#endif // end include guard
