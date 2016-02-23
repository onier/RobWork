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

#ifndef RWSIM_DYNAMICS_OBRMANIFOLD_HPP_
#define RWSIM_DYNAMICS_OBRMANIFOLD_HPP_

#include "ContactPoint.hpp"
#include <rw/math/MetricUtil.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/math/Rotation2D.hpp>

namespace rwsim {
namespace dynamics {
	//! @addtogroup rwsim_dynamics
	//! @{
    /**
     * @brief Contact manifold based on Oriented Bounding Rectangle, so in 2D.
     */
    class OBRManifold {
    public:
        /**
         * @brief
         * @param thres [in] the angle threshold in radians. Threshold of angle between
         * contact normals
         * @param sepThres [in] the max seperating distance in meter between contacting points
         */
        OBRManifold(double thres = 0.03, double sepThres = 0.01):
            _threshold(thres),
            _cosThreshold( fabs( std::cos(thres) ) ),
            _sepThreshold(sepThres),
            _deepestIdx(0),
            _nrOfContacts(0)
        {};

        //! @brief Destructor.
        virtual ~OBRManifold(){};

        /**
         * @brief Adds and updates the manifold with a new point if it fits.
         * @param p [in] the contact point to add.
         * @return true if point fitted inside the manifold, false otherwise.
         */
        bool addPoint(const ContactPoint& p);

        /**
         * @brief Get the deepest penetrating point in the manifold.
         * @return a reference to the point.
         */
        ContactPoint& getDeepestPoint() {
            return _points[_deepestIdx];
        }

        /**
         * @brief Get the deepest penetrating point in the manifold.
         * @return a reference to the point.
         */
        const ContactPoint& getDeepestPoint() const {
            return _points[_deepestIdx];
        }

        void update(const rw::math::Transform3D<> &aT, const rw::math::Transform3D<> &bT){
            using namespace rw::math;
            // update the position of the contact points
            for(int i=0; i<_nrOfContacts; i++){
                const Vector3D<>& wPa = aT * _points[i].pA;
                const Vector3D<>& wPb = bT * _points[i].pB;
                if( MetricUtil::dist2(wPa,wPb)> _sepThreshold ){
                    //remove point
                    _nrOfContacts--;
                    _points[i] = _points[_nrOfContacts];
                    i--;
                } else {
                    _points[i].p = (wPa+wPb)/2;
                }
            }
        }

        /**
         * @brief Get the current number of contacts in the manifold.
         * @return the number of contacts - between 0 and 5.
         */
        int getNrOfContacts() const{ return _nrOfContacts; };

        /**
         * @brief Check if point is in manifold.
         * @param p
         * @return
         */
        bool inManifold(const ContactPoint& p) const {
            using namespace rw::math;
            if( _nrOfContacts==0 ){
                return true;
            } else if( _nrOfContacts == 1){
                // check normal


                // check distance to the deepest point
                const double dist = MetricUtil::dist2(p.p,_points[_deepestIdx].p);
                //std::cout << "1: Dist too point: " << dist << std::endl;
                if( dist <_sepThreshold ) return true;
                else return false;
            } else if( _nrOfContacts == 2 ){
                // check distance to the line
                const double dist = MetricUtil::dist2(p.p,_points[_deepestIdx].p);
                if( dist <_sepThreshold ) return true;
                else return false;
    /*            const Vector3D<> &p1 = p.p;
                const Vector3D<> &x1 = _points[0].p;
                const Vector3D<> &x2 = _points[1].p;
                double dist = MetricUtil::norm2(cross((x2-x1),(x1-p1)))/
                              MetricUtil::norm2((x2-x1));
                if( dist <_threshold ) return true;
                else return false;*/
            } else {
                const double dist = MetricUtil::dist2(p.p,_points[_deepestIdx].p);
                if( dist <_sepThreshold ) return true;
                // check if the point is inside the manifold box
                // project it onto the plane
                return isInsideOBB(p.p);
            }
        }

        /**
         * @brief fits a new manifold to the list of contact points
         * @param p
         */
        void fit(const ContactPoint& p);

        rw::math::Vector3D<> getNormal() const {
            return _normal;
        }

        ContactPoint& getContact(int i) {
            return _points[i];
        }

        const ContactPoint& getContact(int i) const {
            return _points[i];
        }

        rw::math::Transform3D<> getTransform() const { return _t3d;};

        rw::math::Vector3D<> getHalfLengths() const { return _h; };

    private:
        bool isInsideOBB(const rw::math::Vector3D<>& p) const {
            rw::math::Vector3D<> pproj = inverse(_t3d) * p;
            if( fabs(pproj(0))<_h(0) && fabs(pproj(1))<_h(1) ){
                return true;
            }
            return false;
        }

        /**
         * The normal is an average of the normals of the contact normals until 3
         * or more contacts are available. Then the normal becomes the z axis in
         * the OBB that the contacts span
         */
        rw::math::Vector3D<> _normal; //
        //rw::math::Vector3D<> _manifoldNormal; // the normal to the manifold
        rw::math::Transform3D<> _t3d; // transform of the obb
        rw::math::Vector3D<> _h; //halflengths off the obb
        // only used when 3 or more points are available
        ContactPoint _points[5];
        rw::math::Vector2D<> _projPoints[6];
        bool _onBorderMap[4];
        //Frame *_objA,*_objB;
        double _threshold, _cosThreshold, _sepThreshold;
        int _deepestIdx,_nrOfContacts;
    };
    //! @}
}
}

#endif /* RWSIM_DYNAMICS_OBRMANIFOLD_HPP_ */
