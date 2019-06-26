/******************************************************************************
 * Copyright 2019 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
 ******************************************************************************/

#include "QuadraticTestObjects.hpp"

#include <rw/geometry/analytic/quadratics/QuadraticBREP.hpp>
#include <rw/geometry/analytic/quadratics/QuadraticCurve.hpp>
#include <rw/geometry/analytic/quadratics/QuadraticSurface.hpp>
#include <rw/math/RPY.hpp>

using rw::common::ownedPtr;
using namespace rw::geometry;
using rwlibs::geometry::QuadraticTestObjects;
using namespace rw::math;

QuadraticTestObjects::QuadraticTestObjects()
{
}

QuadraticTestObjects::~QuadraticTestObjects()
{
}


QuadraticBREP::Ptr QuadraticTestObjects::objectA() {
    static QuadraticBREP::Ptr brep;
    if (!brep.isNull())
        return brep;

    brep = ownedPtr(new QuadraticBREP());

    const Eigen::DiagonalMatrix<double,3,3> AZero(0,0,0);

    static const Transform3D<> T1(Vector3D<>::zero(), RPY<>(0,0,Pi));
    static const Transform3D<> T2(Vector3D<>(0,0,-10), RPY<>(0,0,-Pi/2));

    const QuadraticSurface::Ptr s1 = QuadraticSurface::makeHyperbolicParaboloid(2,1)->transform(T1);
    const QuadraticSurface::Ptr s2un = QuadraticSurface::makeParabolicCylinder(1);
    const QuadraticSurface::Ptr s2 = QuadraticSurface::makeParabolicCylinder(0.5)->transform(T2);
    QuadraticSurface p3(AZero,Eigen::Vector3d(0,-1,0),-7);
    QuadraticSurface p4(AZero,Eigen::Vector3d(0, 1,0),-7);
    QuadraticSurface p5a(AZero,Eigen::Vector3d(-1,0,0),-8);
    QuadraticSurface p5b = p5a;
    QuadraticSurface p6a(AZero,Eigen::Vector3d( 1,0,0),-8);
    QuadraticSurface p6b = p6a;

    std::vector<QuadraticSurface::TrimmingRegion> trim12;
    trim12.push_back(QuadraticSurface::makePlane(-Vector3D<>::x(),-4));
    trim12.push_back(QuadraticSurface::makePlane( Vector3D<>::x(),-4));
    trim12.push_back(QuadraticSurface::makePlane(-Vector3D<>::y(),-3.5));
    trim12.push_back(QuadraticSurface::makePlane( Vector3D<>::y(),-3.5));

    std::vector<QuadraticSurface::TrimmingRegion> trim34;
    trim34.push_back(QuadraticSurface::makePlane(-Vector3D<>::x(),-4));
    trim34.push_back(QuadraticSurface::makePlane( Vector3D<>::x(),-4));
    {
        Eigen::Matrix3d A = AZero;
        A(0,0) = 0.5;
        trim34.push_back(ownedPtr(new QuadraticSurface(A.selfadjointView<Eigen::Upper>(),Eigen::Vector3d(0,0,-0.5),-10)));
        A(0,0) = 1./8;
        trim34.push_back(ownedPtr(new QuadraticSurface(A.selfadjointView<Eigen::Upper>(),Eigen::Vector3d(0,0, 0.5),-50./8)));
    }

    std::vector<QuadraticSurface::TrimmingRegion> trim56a;
    trim56a.push_back(QuadraticSurface::makePlane(-Vector3D<>::y(),0));
    trim56a.push_back(QuadraticSurface::makePlane( Vector3D<>::y(),-3.5));
    trim56a.push_back(QuadraticSurface::makePlane(-Vector3D<>::z(),-2));
    {
        Eigen::Matrix3d A = AZero;
        A(1,1) = -0.5;
        trim56a.push_back(ownedPtr(new QuadraticSurface(A.selfadjointView<Eigen::Upper>(),Eigen::Vector3d(0,0,0.5),2)));
    }

    std::vector<QuadraticSurface::TrimmingRegion> trim56b;
    trim56b.push_back(QuadraticSurface::makePlane(-Vector3D<>::y(),-3.5));
    trim56b.push_back(QuadraticSurface::makePlane( Vector3D<>::y(),0));
    trim56b.push_back(trim56a[2]);
    trim56b.push_back(trim56a[3]);

    s1->setTrimmingConditions(trim12);
    s2->setTrimmingConditions(trim12);
    p3.setTrimmingConditions(trim34);
    p4.setTrimmingConditions(trim34);
    p5a.setTrimmingConditions(trim56a);
    p5b.setTrimmingConditions(trim56b);
    p6a.setTrimmingConditions(trim56a);
    p6b.setTrimmingConditions(trim56b);

    QuadraticCurve c1(Vector3D<>(0,-3.5,49./8),-4.*Vector3D<>::x(),-2.*Vector3D<>::z(),QuadraticCurve::Parabola);
    QuadraticCurve c2(Vector3D<>(0, 3.5,49./8),-4.*Vector3D<>::x(),-2.*Vector3D<>::z(),QuadraticCurve::Parabola);
    QuadraticCurve c3a(Vector3D<>(-4, 0, -2), 3.5*Vector3D<>::y(),49./8*Vector3D<>::z(),QuadraticCurve::Parabola);
    QuadraticCurve c3b(Vector3D<>(-4, 0, -2),-3.5*Vector3D<>::y(),49./8*Vector3D<>::z(),QuadraticCurve::Parabola);
    QuadraticCurve c4a(Vector3D<>( 4, 0, -2), 3.5*Vector3D<>::y(),49./8*Vector3D<>::z(),QuadraticCurve::Parabola);
    QuadraticCurve c4b(Vector3D<>( 4, 0, -2),-3.5*Vector3D<>::y(),49./8*Vector3D<>::z(),QuadraticCurve::Parabola);
    QuadraticCurve c5(Vector3D<>(0,-3.5,-10),-4.*Vector3D<>::x(),8.*Vector3D<>::z(),QuadraticCurve::Parabola);
    QuadraticCurve c6(Vector3D<>(0, 3.5,-10),-4.*Vector3D<>::x(),8.*Vector3D<>::z(),QuadraticCurve::Parabola);

    c1.setLimits(std::make_pair(-1.,1.));
    c2.setLimits(std::make_pair(-1.,1.));
    c3a.setLimits(std::make_pair(0.,1.));
    c3b.setLimits(std::make_pair(0.,1.));
    c4a.setLimits(std::make_pair(0.,1.));
    c4b.setLimits(std::make_pair(0.,1.));
    c5.setLimits(std::make_pair(-1.,1.));
    c6.setLimits(std::make_pair(-1.,1.));

    QuadraticCurve l7a(Vector3D<>(-4, 0, -2),-3.5*Vector3D<>::y(),Vector3D<>::z(),QuadraticCurve::Line);
    QuadraticCurve l7b(Vector3D<>(-4, 0, -2), 3.5*Vector3D<>::y(),Vector3D<>::z(),QuadraticCurve::Line);
    QuadraticCurve l8a(Vector3D<>( 4, 0, -2),-3.5*Vector3D<>::y(),Vector3D<>::z(),QuadraticCurve::Line);
    QuadraticCurve l8b(Vector3D<>( 4, 0, -2), 3.5*Vector3D<>::y(),Vector3D<>::z(),QuadraticCurve::Line);
    QuadraticCurve l9( Vector3D<>(-4,-3.5,-2),49./8*Vector3D<>::z(),Vector3D<>::y(),QuadraticCurve::Line);
    QuadraticCurve l10(Vector3D<>( 4,-3.5,-2),49./8*Vector3D<>::z(),Vector3D<>::y(),QuadraticCurve::Line);
    QuadraticCurve l11(Vector3D<>(-4, 3.5,-2),49./8*Vector3D<>::z(),Vector3D<>::y(),QuadraticCurve::Line);
    QuadraticCurve l12(Vector3D<>( 4, 3.5,-2),49./8*Vector3D<>::z(),Vector3D<>::y(),QuadraticCurve::Line);

    l7a.setLimits(std::make_pair(0.,1.));
    l7b.setLimits(std::make_pair(0.,1.));
    l8a.setLimits(std::make_pair(0.,1.));
    l8b.setLimits(std::make_pair(0.,1.));
    l9.setLimits(std::make_pair(0.,1.));
    l10.setLimits(std::make_pair(0.,1.));
    l11.setLimits(std::make_pair(0.,1.));
    l12.setLimits(std::make_pair(0.,1.));

    const Vector3D<> v0(-4,-3.5,33./8);
    const Vector3D<> v1( 4,-3.5,33./8);
    const Vector3D<> v2(-4,-3.5,33./8);
    const Vector3D<> v3( 4,-3.5,33./8);
    const Vector3D<> v4(-4, 0,  -2);
    const Vector3D<> v5( 4, 0,  -2);
    const Vector3D<> v6(-4,-3.5,-2);
    const Vector3D<> v7(-4, 3.5,-2);
    const Vector3D<> v8( 4,-3.5,-2);
    const Vector3D<> v9(4, 3.5,-2);

    brep->addVertex(v0);
    brep->addVertex(v1);
    brep->addVertex(v2);
    brep->addVertex(v3);
    brep->addVertex(v4);
    brep->addVertex(v5);
    brep->addVertex(v6);
    brep->addVertex(v7);
    brep->addVertex(v8);
    brep->addVertex(v9);

    brep->addEdge(c1,1,0);  // 1
    brep->addEdge(c2,3,2);  // 2
    brep->addEdge(c3a,8,2); // 3
    brep->addEdge(c3b,8,0); // 4
    brep->addEdge(c4a,9,3); // 5
    brep->addEdge(c4b,9,1); // 6
    brep->addEdge(c5,6,4);  // 7
    brep->addEdge(c6,7,5);  // 8
    brep->addEdge(l7a,8,4); // 9
    brep->addEdge(l7b,8,5); // 10
    brep->addEdge(l8a,9,6); // 11
    brep->addEdge(l8b,9,7); // 12
    brep->addEdge(l9,4,0);  // 13
    brep->addEdge(l10,6,1); // 14
    brep->addEdge(l11,5,2); // 15
    brep->addEdge(l12,7,3); // 16

    brep->makeLoop(-1,-6,5,2,-3,4); // top (sadle)
    brep->makeLoop(7,-9,10,-8,-12,11); // bottom
    brep->makeLoop(1,-13,-7,14); // front
    brep->makeLoop(-2,-16,8,15); // back
    brep->makeLoop(3,-15,-10); // left back
    brep->makeLoop(-4,9,13); // left front
    brep->makeLoop(-5,12,16); // right back
    brep->makeLoop(6,-14,-11); // right front

    brep->setFace(*s1,0);
    brep->setFace(*s2,1);
    brep->setFace(p3,2);
    brep->setFace(p4,3);
    brep->setFace(p5a,4);
    brep->setFace(p5b,5);
    brep->setFace(p6a,6);
    brep->setFace(p6b,7);

    return brep;
}

QuadraticBREP::Ptr QuadraticTestObjects::objectB() {
    static QuadraticBREP::Ptr brep;
    if (!brep.isNull())
        return brep;

    brep = ownedPtr(new QuadraticBREP());

    const Eigen::DiagonalMatrix<double,3,3> AZero(0,0,0);

    static const QuadraticSurface::Ptr s1 = QuadraticSurface::makeEllipticCylinder(3,4)->transform(Transform3D<>(Vector3D<>(0,0,5)));
    static const QuadraticSurface::Ptr s2 = QuadraticSurface::makeEllipticHyperboloidOneSheet(std::sqrt(576./137),4,8)->transform(Transform3D<>(Vector3D<>::zero(), RPY<>(0,-Pi/4,0)));
    static const QuadraticSurface::Ptr s3 = QuadraticSurface::makeEllipticHyperboloidOneSheet(std::sqrt(576./137),4,8)->transform(Transform3D<>(Vector3D<>::zero(), RPY<>(0, Pi/4,0)));
    static const QuadraticSurface::Ptr s4 = QuadraticSurface::makeEllipticParaboloid(1,4./3)->transform(Transform3D<>(Vector3D<>(0,0,-10)));
    static const QuadraticSurface::Ptr p5 = QuadraticSurface(AZero,Eigen::Vector3d( 1,0,0),-15).transform(Transform3D<>(Vector3D<>::zero(),RPY<>(0,-Pi/4,0))*Transform3D<>(Vector3D<>(0,0,5)));
    static const QuadraticSurface::Ptr p6 = QuadraticSurface(AZero,Eigen::Vector3d(-1,0,0),-15).transform(Transform3D<>(Vector3D<>::zero(),RPY<>(0,Pi/4,0))*Transform3D<>(Vector3D<>(0,0,5)));

    std::vector<QuadraticSurface::TrimmingRegion> trim1;
    trim1.push_back(QuadraticSurface::makePlane(-Vector3D<>::z(),5.5));
    trim1.push_back(QuadraticSurface::makePlane( Vector3D<>::z(),0));

    std::vector<QuadraticSurface::TrimmingRegion> trim2;
    trim2.push_back(p6);
    trim2.push_back(QuadraticSurface::makePlane( Vector3D<>::x(),0));
    trim2.push_back(QuadraticSurface::makePlane(-Vector3D<>::z(),0));

    std::vector<QuadraticSurface::TrimmingRegion> trim3;
    trim2.push_back(p5);
    trim2.push_back(QuadraticSurface::makePlane(-Vector3D<>::x(),0));
    trim2.push_back(QuadraticSurface::makePlane(-Vector3D<>::z(),0));

    std::vector<QuadraticSurface::TrimmingRegion> trim4;
    trim4.push_back(QuadraticSurface::makePlane(Vector3D<>::z(),-5.5));

    s1->setTrimmingConditions(trim1);
    s2->setTrimmingConditions(trim2);
    s3->setTrimmingConditions(trim3);
    s4->setTrimmingConditions(trim4);

    static const double k1 = std::sqrt(2)*3*685/548;
    static const double k2 = std::sqrt(131794)*3/548;
    static const double k3 = std::sqrt(481./16);
    QuadraticCurve c1_2(Vector3D<>::zero(), Vector3D<>::y()*4,Vector3D<>::x()*3,QuadraticCurve::Elliptic);
    QuadraticCurve c1_3 = c1_2;
    QuadraticCurve c1_4(Vector3D<>(0,0,-5.5),Vector3D<>::y()*4,Vector3D<>::x()*3,QuadraticCurve::Elliptic);
    QuadraticCurve c2_3(Vector3D<>::zero(),Vector3D<>::z()*3,Vector3D<>::y()*4,QuadraticCurve::Elliptic);
    QuadraticCurve c2_6(Vector3D<>(-1,0,1)*k1,Vector3D<>(1,0, 1)*k2,Vector3D<>(0,1,0)*k3,QuadraticCurve::Elliptic);
    QuadraticCurve c3_5(Vector3D<>( 1,0,1)*k1,Vector3D<>(1,0,-1)*k2,Vector3D<>(0,1,0)*k3,QuadraticCurve::Elliptic);
    QuadraticCurve c2(Vector3D<>::zero(),normalize(Vector3D<>(-1,0,1))*8,Vector3D<>::y()*4,QuadraticCurve::Hyperbola);
    QuadraticCurve c3(Vector3D<>::zero(),normalize(Vector3D<>( 1,0,1))*8,Vector3D<>::y()*4,QuadraticCurve::Hyperbola);
    QuadraticCurve c4(Vector3D<>(0,-4,0),-5.5*Vector3D<>::z(),Vector3D<>::zero(),QuadraticCurve::Line);

    c1_2.setLimits(std::make_pair( Pi/2,Pi*3/2));
    c1_3.setLimits(std::make_pair(-Pi/2.,Pi/2));
    c1_4.setLimits(std::make_pair(-Pi/2,Pi*3/2));
    c2_3.setLimits(std::make_pair(0.,Pi));
    c2_6.setLimits(std::make_pair(0.,2*Pi));
    c3_5.setLimits(std::make_pair(0.,2*Pi));
    c2.setLimits(std::make_pair(0.,std::atanh(15./std::sqrt(481))));
    c3.setLimits(std::make_pair(0.,std::atanh(15./std::sqrt(481))));
    c4.setLimits(std::make_pair(0.,1.));

    const Vector3D<> v0(0, 4, 0);
    const Vector3D<> v1(0,-4, 0);
    const Vector3D<> v2(-15./(2.*std::sqrt(2)), std::sqrt(481)/4., 15./(2.*std::sqrt(2)));
    const Vector3D<> v3( 15./(2.*std::sqrt(2)), std::sqrt(481)/4., 15./(2.*std::sqrt(2)));
    const Vector3D<> v4(0,-4,-3.5);

    brep->addVertex(v0);
    brep->addVertex(v1);
    brep->addVertex(v2);
    brep->addVertex(v3);
    brep->addVertex(v4);

    brep->addEdge(c1_2,0,1);  // 1
    brep->addEdge(c1_3,1,0);  // 2
    brep->addEdge(c1_4,4,4);  // 3
    brep->addEdge(c2_3,0,1); // 4
    brep->addEdge(c2_6,2,2); // 5
    brep->addEdge(c3_5,3,3); // 6
    brep->addEdge(c2,0,2); // 7
    brep->addEdge(c3,0,3); // 8
    brep->addEdge(c4,1,4); // 9

    brep->makeLoop(-2,9,3,-9,-1,-2); // face 1
    brep->makeLoop(7,5,-7,1,-4,7); // face 2
    brep->makeLoop(4,2,8,6,-8,4); // face 3
    brep->makeLoop(-3); // face 4
    brep->makeLoop(-6); // face 5
    brep->makeLoop(-5); // face 6

    brep->setFace(*s1,0);
    brep->setFace(*s2,1);
    brep->setFace(*s3,2);
    brep->setFace(*s4,3);
    brep->setFace(*p5,4);
    brep->setFace(*p6,5);

    return brep;
}
