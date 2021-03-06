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

#include "WrenchMeasure3D.hpp"

#include <rw/math/EAA.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Constants.hpp>
#include <boost/foreach.hpp>
#include <iostream>

using namespace rw::math;
using namespace rw::geometry;
using namespace rw::graspplanning::sandbox;


namespace {

    /**
     * calculate the normalized friction cone from a force and friction
     * @param force
     * @param mu
     * @return
     */
    std::vector<Vector3D<> > getCone(const Vector3D<>& normal, double nforce, double mu, int resolution){
        Vector3D<> unormal = normalize(normal);
        //double angle = atan(mu);
        //tan(angle)*1
        // create a vector that is tangent to the normal
        const double EPSILON = 0.000001;
        Vector3D<> tdir;
        if( fabs(normal(0))<EPSILON && fabs(normal(1))<EPSILON ){
            tdir = normalize(  Vector3D<>(0,-normal(2),normal(1)) );
        } else {
            tdir = normalize(  Vector3D<>(-normal(1),normal(0),0) );
        }
        Vector3D<> tdirScaled = tdir * (nforce * mu);

        Vector3D<> fnormal = unormal * nforce;
        std::vector<Vector3D<> > coneVerts;
        // rotate the vector around the normal in some fixed steps
        Vector3D<> P = tdirScaled; // vector to rotate
        Vector3D<> A = unormal; // vector to rotate about

        EAA<> eaa(unormal, 2*Pi/resolution );
        Rotation3D<> rot( eaa.toRotation3D() );
        for(int i=0; i<resolution-1; i++){
            coneVerts.push_back(fnormal+P);
            P = rot*P;
        }
        return coneVerts;
    }

}

WrenchMeasure3D::WrenchMeasure3D(int resolution, bool useUnitVectors):
    _chullCalculator( rw::common::ownedPtr(new QHullND<6>() ) ),
    _resolution(resolution),
    _useUnitVectors(useUnitVectors),
    _lambda(1.0/0.1)
{
}


double WrenchMeasure3D::quality(const rw::graspplanning::Grasp3D& grasp) const {
    std::vector< QHullND<6>::VectorND > vertices;

    BOOST_FOREACH(const rw::sensor::Contact3D& c, grasp.contacts ){
        // std::cout  << "get cone: " << c.n << " " << c.normalForce << std::endl;
         if(c.normalForce<0.0001){
        	 RW_WARN("Normal force too small! : " << c.normalForce);
        	 continue;
         }

    	 Vector3D<> arm = c.p - _objCenter;
    	 double normalForce = 1.0;
    	 if(!_useUnitVectors)
    	     normalForce = c.normalForce;
         std::vector<Vector3D<> > verts = getCone(c.n,normalForce,c.mu,_resolution);
         BOOST_FOREACH(const Vector3D<> &force, verts){
             Vector3D<> torque =  cross(arm,force);
             QHullND<6>::VectorND vertice;
             vertice[0] = force[0];
             vertice[1] = force[1];
             vertice[2] = force[2];
             vertice[3] = _lambda * torque[0];
             vertice[4] = _lambda * torque[1];
             vertice[5] = _lambda * torque[2];
             vertices.push_back(vertice);
         }
    }



    // first do the force space
    _chullCalculator->rebuild( vertices );

    QHullND<6>::VectorND origin;
    for(int i=0;i<6;i++) origin[i] = 0;

    // test if the center is inside
    //_isInside = _chullCalculator->isInside( origin );
    _minWrench = _chullCalculator->getMinDistInside( origin, vertices);
    _isInside = _minWrench>=0;

    return _minWrench;
}
