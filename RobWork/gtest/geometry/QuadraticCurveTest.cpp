/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <gtest/gtest.h>

#include <rw/geometry/analytic/quadratics/QuadraticCurve.hpp>
#include <rw/geometry/analytic/quadratics/QuadraticUtil.hpp>

using namespace rw::geometry;
using namespace rw::math;

TEST(QuadraticCurve, Generic) {
	QuadraticCurve curve(Vector3D<>(1.7,2.1,3.2),Vector3D<>(-0.45,0.57,std::sqrt(4726.)/100)*6,Vector3D<>(0,-std::sqrt(4726./7975.),std::sqrt(3249./7975.))*4,QuadraticCurve::Parabola);

	// Accessors
	EXPECT_EQ(1.7,curve.c()[0]);
	EXPECT_EQ(2.1,curve.c()[1]);
	EXPECT_EQ(3.2,curve.c()[2]);

	EXPECT_EQ(-2.7,curve.u()[0]);
	EXPECT_EQ(3.42,curve.u()[1]);
	EXPECT_EQ(6.*std::sqrt(4726.)/100,curve.u()[2]);

	EXPECT_EQ(0,curve.v()[0]);
	EXPECT_EQ(-4.*std::sqrt(4726./7975.),curve.v()[1]);
	EXPECT_EQ(4.*std::sqrt(3249./7975.),curve.v()[2]);

	EXPECT_FALSE(curve.hasLimits());
	curve.setLimits(std::make_pair(-0.11,0.22));
	EXPECT_TRUE(curve.hasLimits());

	EXPECT_DOUBLE_EQ(-0.11,curve.limits().first);
	EXPECT_DOUBLE_EQ(0.22,curve.limits().second);
}

TEST(QuadraticCurve, Line) {
	QuadraticCurve curve(Vector3D<>(1.7,2.1,3.2),Vector3D<>(-2.9,3.5,4.1),Vector3D<>(0,-4.1,3.5),QuadraticCurve::Line);

	EXPECT_EQ(1.7, curve.x(0)[0]);
	EXPECT_EQ(2.1, curve.x(0)[1]);
	EXPECT_EQ(3.2, curve.x(0)[2]);

	EXPECT_EQ(0.25, curve.x(0.5)[0]);
	EXPECT_EQ(3.85, curve.x(0.5)[1]);
	EXPECT_EQ(5.25, curve.x(0.5)[2]);

	EXPECT_EQ(0.25, curve(0.5)[0]);
	EXPECT_EQ(3.85, curve(0.5)[1]);
	EXPECT_EQ(5.25, curve(0.5)[2]);

	EXPECT_EQ(-2.9, curve.dx(0.5)[0]);
	EXPECT_EQ(3.5, curve.dx(0.5)[1]);
	EXPECT_EQ(4.1, curve.dx(0.5)[2]);

	EXPECT_EQ(0, curve.ddx(0.5)[0]);
	EXPECT_EQ(0, curve.ddx(0.5)[1]);
	EXPECT_EQ(0, curve.ddx(0.5)[2]);

	EXPECT_EQ(QuadraticCurve::Line, curve.type());

	EXPECT_EQ(std::size_t(2), curve.discretizeAdaptive(0).size());
	EXPECT_EQ(std::size_t(2), curve.discretizeAdaptive(10).size());
	EXPECT_EQ(std::size_t(2), curve.discretizeAdaptive(100).size());

	std::vector<Vector3D<> > closest;
	std::vector<double> times;

	closest = curve.closestPoints(Vector3D<>(9,1,5));
	times = curve.closestTimes(Vector3D<>(9,1,5));
	ASSERT_EQ(std::size_t(1), closest.size());
	ASSERT_EQ(std::size_t(1), times.size());
	EXPECT_DOUBLE_EQ(-588./1249., times[0]); // t=(p-c)^T/(u^Tu)
	EXPECT_DOUBLE_EQ(7657./2498., closest[0][0]);
	EXPECT_DOUBLE_EQ(5649./12490., closest[0][1]);
	EXPECT_DOUBLE_EQ(1586./1249., closest[0][2]);

	curve.setLimits(std::make_pair(2,3));

	closest = curve.closestPoints(Vector3D<>(9,1,5));
	times = curve.closestTimes(Vector3D<>(9,1,5));
	ASSERT_EQ(std::size_t(1), closest.size());
	ASSERT_EQ(std::size_t(1), times.size());
	EXPECT_DOUBLE_EQ(2., times[0]);
	EXPECT_DOUBLE_EQ(-4.1, closest[0][0]);
	EXPECT_DOUBLE_EQ(9.1, closest[0][1]);
	EXPECT_DOUBLE_EQ(11.4, closest[0][2]);

	closest = curve.closestPoints(Vector3D<>(-9,10,20));
	times = curve.closestTimes(Vector3D<>(-9,10,20));
	ASSERT_EQ(std::size_t(1), closest.size());
	ASSERT_EQ(std::size_t(1), times.size());
	EXPECT_DOUBLE_EQ(3., times[0]);
	EXPECT_DOUBLE_EQ(-7., closest[0][0]);
	EXPECT_DOUBLE_EQ(12.6, closest[0][1]);
	EXPECT_DOUBLE_EQ(15.5, closest[0][2]);

	EXPECT_EQ(0,curve.curvature(2));

	const OBB<> obb = curve.obr();
	EXPECT_EQ(-5.55, obb.getTransform().P()[0]);
	EXPECT_EQ(10.85, obb.getTransform().P()[1]);
	EXPECT_EQ(13.45, obb.getTransform().P()[2]);
	EXPECT_EQ(std::sqrt(37.47)/2., obb.getHalfLengths()[0]);
	EXPECT_EQ(0, obb.getHalfLengths()[1]);
	EXPECT_EQ(0, obb.getHalfLengths()[2]);
}

TEST(QuadraticCurve, Parabola) {
	QuadraticCurve curve(Vector3D<>(1.7,2.1,3.2),Vector3D<>(-0.45,0.57,std::sqrt(4726.)/100)*6,Vector3D<>(0,-std::sqrt(4726./7975.),std::sqrt(3249./7975.))*4,QuadraticCurve::Parabola);

	EXPECT_EQ(2./9, curve.curvature(0));

	{
		const std::vector<double> closestTimes = curve.closestTimes(Vector3D<>(5, -7, 3.2));
		ASSERT_EQ(std::size_t(1),closestTimes.size());
		EXPECT_DOUBLE_EQ(-1.2695321620177378,closestTimes[0]);
	}
	curve.setLimits(std::make_pair(-1.0,0.5));
	{
		const std::vector<double> closestTimes = curve.closestTimes(Vector3D<>(5, -7, 3.2));
		ASSERT_EQ(std::size_t(1),closestTimes.size());
		EXPECT_EQ(-1.,closestTimes[0]);
		EXPECT_EQ(-1.,curve.closestTime(Vector3D<>(5, -7, 3.2)));
		const std::vector<Vector3D<> > closestPoints = curve.closestPoints(Vector3D<>(5, -7, 3.2));
		ASSERT_EQ(closestPoints.size(), closestTimes.size());
		EXPECT_EQ(0.,(closestPoints[0]-curve.x(closestTimes[0])).norm2());
	}

	{
		const std::list<Vector3D<> > discretization = curve.discretizeAdaptive(100);
		bool pointAtLowerLimit = false;
		bool pointAtUpperLimit = false;
		bool pointAtExtremum = false;
		for (std::list<Vector3D<> >::const_iterator it = discretization.begin(); it != discretization.end(); it++) {
			if (((*it)-curve(curve.limits().first)).norm2() < 1e-14)
				pointAtLowerLimit = true;
			if (((*it)-curve(curve.limits().second)).norm2() < 1e-14)
				pointAtUpperLimit = true;
			if (((*it)-curve(0)).norm2() < 1e-14)
				pointAtExtremum = true;
		}
		EXPECT_TRUE(pointAtLowerLimit);
		EXPECT_TRUE(pointAtUpperLimit);
		EXPECT_TRUE(pointAtExtremum);
		EXPECT_EQ(std::size_t(6),discretization.size());
	}
}

TEST(QuadraticCurve, Ellipse) {
	QuadraticCurve curve(Vector3D<>(1.7,2.1,3.2),Vector3D<>(-0.45,0.57,std::sqrt(4726.)/100)*6,Vector3D<>(0,-std::sqrt(4726./7975.),std::sqrt(3249./7975.))*4,QuadraticCurve::Elliptic);

	EXPECT_EQ(1./9, curve.curvature(0));
	EXPECT_EQ(3./8, curve.curvature(Pi/2));
	EXPECT_EQ(1./9, curve.curvature(Pi));
	EXPECT_EQ(3./8, curve.curvature(Pi*3/2));


    QuadraticCurve c1(Vector3D<>(1.1,2.2,3.3), -Vector3D<>::x()*0.001, Vector3D<>::y()*0.001, QuadraticCurve::Elliptic);
    QuadraticCurve c2(Vector3D<>(1.1,2.2,3.3),  Vector3D<>::x()*0.001, Vector3D<>::y()*0.001, QuadraticCurve::Elliptic);
    c1.setLimits(std::make_pair(-Pi/2,0.));
    c2.setLimits(std::make_pair(0.,Pi/2));

    const std::list<Vector3D<> > p1 = c1.discretizeAdaptive(10);
    const std::list<Vector3D<> > p2 = c2.discretizeAdaptive(10);

    ASSERT_EQ(std::size_t(4), p1.size());
    ASSERT_EQ(std::size_t(4), p2.size());

    std::list<Vector3D<> >::const_iterator i1 = p1.begin();
    std::list<Vector3D<> >::const_reverse_iterator i2 = p2.rbegin();

    EXPECT_DOUBLE_EQ(0., (*i1++ - *i2++).normInf());
    EXPECT_DOUBLE_EQ(0., (*i1++ - *i2++).normInf());
    EXPECT_DOUBLE_EQ(0., (*i1++ - *i2++).normInf());
    EXPECT_DOUBLE_EQ(0., (*i1++ - *i2++).normInf());

    EXPECT_DOUBLE_EQ(0., c2.closestTime(Vector3D<>(1.1,2.201,3.3)));
    EXPECT_NEAR(-Pi/4, c1.closestTime(Vector3D<>(1.10075,2.20075,3.3)), 2e-13);
    EXPECT_NEAR(Pi/4, c2.closestTime(Vector3D<>(1.10075,2.20075,3.3)), 2e-13);
    EXPECT_NEAR(Pi/4, c2.closestTime(Vector3D<>(1.4,2.5,3.3)), 2e-13);
    EXPECT_DOUBLE_EQ(Pi/2, c2.closestTime(Vector3D<>(1.101,2.2,3.3)));
    c2.setLimits(std::make_pair(-Pi,Pi));
    EXPECT_NEAR(3.*Pi/4, c2.closestTime(Vector3D<>(1.10075,2.19925,3.3)), 2e-13);
    EXPECT_DOUBLE_EQ(Pi, c2.closestTime(Vector3D<>(1.1,2.19925,3.3)));
}

TEST(QuadraticCurve, Hyperbola) {
	QuadraticCurve curve(Vector3D<>(1.7,2.1,3.2),Vector3D<>(-0.45,0.57,std::sqrt(4726.)/100)*6,Vector3D<>(0,-std::sqrt(4726./7975.),std::sqrt(3249./7975.))*4,QuadraticCurve::Hyperbola);
	EXPECT_EQ(QuadraticCurve::Hyperbola,curve.type());

	EXPECT_EQ(1./9, curve.curvature(0));

	{
		const Vector3D<> eval1 = curve(-std::log(2));
		EXPECT_DOUBLE_EQ(149./40,eval1[0]);
		EXPECT_DOUBLE_EQ(-93./200-5.*std::sqrt(4726./7975),eval1[1]);
		EXPECT_DOUBLE_EQ(3.2-9./200*std::sqrt(4726.)+5.*std::sqrt(3249./7975),eval1[2]);

		const Vector3D<> eval2 = curve.x(-std::log(2));
		EXPECT_DOUBLE_EQ(eval1[0],eval2[0]);
		EXPECT_DOUBLE_EQ(eval1[1],eval2[1]);
		EXPECT_DOUBLE_EQ(eval1[2],eval2[2]);

		const Vector3D<> dx = curve.dx(-std::log(2));
		EXPECT_DOUBLE_EQ(-27./8,dx[0]);
		EXPECT_DOUBLE_EQ(171./40+3.*std::sqrt(4726./7975),dx[1]);
		EXPECT_DOUBLE_EQ(3./40*std::sqrt(4726.)-3.*std::sqrt(3249./7975),dx[2]);

		const Vector3D<> ddx = curve.ddx(-std::log(2));
		EXPECT_DOUBLE_EQ(81./40,ddx[0]);
		EXPECT_DOUBLE_EQ(-513./200-5.*std::sqrt(4726./7975),ddx[1]);
		EXPECT_DOUBLE_EQ(-9./200*std::sqrt(4726.)+5.*std::sqrt(3249./7975),ddx[2]);
	}

	{
		const std::vector<double> closestTimes = curve.closestTimes(Vector3D<>(5, -7, 3.2));
		ASSERT_EQ(std::size_t(1),closestTimes.size());
		EXPECT_DOUBLE_EQ(-1.003819869316751,closestTimes[0]);
	}
	curve.setLimits(std::make_pair(-std::log(2.),1.0));
	{
		const std::vector<double> closestTimes = curve.closestTimes(Vector3D<>(5, -7, 3.2));
		ASSERT_EQ(std::size_t(1),closestTimes.size());
		EXPECT_EQ(-std::log(2.),closestTimes[0]);
		EXPECT_EQ(-std::log(2.),curve.closestTime(Vector3D<>(5, -7, 3.2)));
		const std::vector<Vector3D<> > closestPoints = curve.closestPoints(Vector3D<>(5, -7, 3.2));
		ASSERT_EQ(closestPoints.size(), closestTimes.size());
		EXPECT_EQ(0.,(closestPoints[0]-curve.x(closestTimes[0])).norm2());
	}

	const OBB<> obb = curve.obr();
	EXPECT_NEAR(1.0919998842108387, obb.getTransform().P()[0],1e-15);
	EXPECT_NEAR(-0.8573944458513321, obb.getTransform().P()[1],1e-15);
	EXPECT_NEAR(7.219477219639269, obb.getTransform().P()[2],1e-15);
	EXPECT_DOUBLE_EQ(((std::exp(1.)*2-1)*std::sqrt(std::exp(1.)*20 + std::exp(2.)*13+52))/(std::exp(1.)*4), obb.getHalfLengths()[0]);
	EXPECT_DOUBLE_EQ(((std::exp(1.)*2 - std::sqrt(std::exp(1.)*2)*2+1)*6)/std::sqrt(std::exp(1.)*20 + std::exp(2.)*13+52), obb.getHalfLengths()[1]);
	EXPECT_DOUBLE_EQ(0, obb.getHalfLengths()[2]);

	{
		const std::list<Vector3D<> > discretization = curve.discretizeAdaptive(100);
		bool pointAtLowerLimit = false;
		bool pointAtUpperLimit = false;
		bool pointAtExtremum = false;
		for (std::list<Vector3D<> >::const_iterator it = discretization.begin(); it != discretization.end(); it++) {
			if (((*it)-curve(curve.limits().first)).norm2() < 1e-14)
				pointAtLowerLimit = true;
			if (((*it)-curve(curve.limits().second)).norm2() < 1e-14)
				pointAtUpperLimit = true;
			if (((*it)-curve(0)).norm2() < 1e-14)
				pointAtExtremum = true;
		}
		EXPECT_TRUE(pointAtLowerLimit);
		EXPECT_TRUE(pointAtUpperLimit);
		EXPECT_TRUE(pointAtExtremum);
		EXPECT_EQ(std::size_t(5),discretization.size());
	}
}
