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

#include <rw/geometry/Polygon.hpp>
#include <rw/geometry/PolygonUtil.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>

using rw::geometry::Polygon;
using rw::geometry::PolygonUtil;
using namespace rw::math;

TEST(Polygon, Vector3D) {
	Polygon<> polygon;
	polygon.addVertex(Vector3D<>(1.,0.,-0.1));
	polygon.addVertex(Vector3D<>(100.,100.,100.));
	polygon.addVertex(Vector3D<>(-1.,1.,-0.1));
	polygon.addVertex(Vector3D<>(-1.,-1.,-0.1));
	EXPECT_EQ(std::size_t(4),polygon.size());
	polygon.removeVertex(1);
	EXPECT_EQ(std::size_t(3),polygon.size());
	EXPECT_DOUBLE_EQ(-1./3.,polygon.computeCenter()[0]);
	EXPECT_DOUBLE_EQ(0.,polygon.computeCenter()[1]);
	EXPECT_DOUBLE_EQ(-0.1,polygon.computeCenter()[2]);
	EXPECT_EQ(1.,polygon.getVertex(0)[0]);
	EXPECT_EQ(0.,polygon.getVertex(0)[1]);
	EXPECT_EQ(-0.1,polygon.getVertex(0)[2]);
	EXPECT_EQ(-1.,polygon[2][0]);
	EXPECT_EQ(-1.,polygon[2][1]);
	EXPECT_EQ(-0.1,polygon[2][2]);
}

TEST(Polygon, Vector2D) {
	Polygon<Vector2D<> > polygon;
	polygon.addVertex(Vector2D<>(1.,0.));
	polygon.addVertex(Vector2D<>(100.,100.));
	polygon.addVertex(Vector2D<>(-1.,1.));
	polygon.addVertex(Vector2D<>(-1.,-1.));
	EXPECT_EQ(std::size_t(4),polygon.size());
	polygon.removeVertex(1);
	EXPECT_EQ(std::size_t(3),polygon.size());
	EXPECT_DOUBLE_EQ(-1./3.,polygon.computeCenter()[0]);
	EXPECT_DOUBLE_EQ(0.,polygon.computeCenter()[1]);
	EXPECT_EQ(1.,polygon.getVertex(0)[0]);
	EXPECT_EQ(0.,polygon.getVertex(0)[1]);
	EXPECT_EQ(-1.,polygon[2][0]);
	EXPECT_EQ(-1.,polygon[2][1]);
}

TEST(DeathTest, Polygon) {
	::testing::FLAGS_gtest_death_test_style = "threadsafe";
	Polygon<> polygon;
	polygon.addVertex(Vector3D<>(1.,0.,-0.1));
	polygon.addVertex(Vector3D<>(100.,100.,100.));
	polygon.addVertex(Vector3D<>(-1.,1.,-0.1));
	polygon.addVertex(Vector3D<>(-1.,-1.,-0.1));
	EXPECT_DEATH_IF_SUPPORTED(polygon.removeVertex(4),"Polygon.hpp:*");
	EXPECT_DEATH_IF_SUPPORTED(polygon.getVertex(4),"Polygon.hpp:*");
}

namespace {
    void testPerturbation(const bool perturbed) {
        // perturbation adds up to 3.33e-16 to point coordinates
        Polygon<Vector2D<> > polygon;
        polygon.addVertex(Vector2D<>(7.50000000000000222e-01, -5.48292804986532767e-01));
        polygon.addVertex(Vector2D<>(7.50000000000000222e-01, -5.40836573825408840e-01));
        polygon.addVertex(Vector2D<>(7.50000000000000222e-01, -5.15176841951355935e-01));
        polygon.addVertex(Vector2D<>(7.50000000000000222e-01, -4.51844908134717005e-01));
        if (perturbed)
            polygon.addVertex(Vector2D<>(7.50000000000000222e-01, -2.80070874150444460e-01));
            else
                polygon.addVertex(Vector2D<>(7.50000000000000222e-01, -2.80070874150444515e-01));
        polygon.addVertex(Vector2D<>(7.50000000000000222e-01, -1.00719754293342193e-16));
        polygon.addVertex(Vector2D<>(7.50000000000000222e-01, 2.80070874150443072e-01));
        polygon.addVertex(Vector2D<>(7.50000000000000222e-01, 4.51844908134716616e-01));
        if (perturbed)
            polygon.addVertex(Vector2D<>(7.50000000000000111e-01, 5.15176841951355824e-01));
        else
            polygon.addVertex(Vector2D<>(7.50000000000000222e-01, 5.15176841951355824e-01));
        polygon.addVertex(Vector2D<>(7.50000000000000222e-01, 5.40836573825408728e-01));
        if (perturbed)
            polygon.addVertex(Vector2D<>(7.49999999999999667e-01, 5.48292804986532656e-01));
        else
            polygon.addVertex(Vector2D<>(7.50000000000000000e-01, 5.48292804986532767e-01));
        polygon.addVertex(Vector2D<>(-1.29893408435323999e-17, 4.00000000000000022e-01));
        polygon.addVertex(Vector2D<>(7.34225154431479704e-02, 3.75276534368993664e-01));
        polygon.addVertex(Vector2D<>(1.45740200767925959e-01, 2.90653173265546905e-01));
        polygon.addVertex(Vector2D<>(2.05618020776569627e-01, 9.83640319435003657e-02));
        polygon.addVertex(Vector2D<>(2.12132034355964283e-01, 4.89858719658941308e-17));
        polygon.addVertex(Vector2D<>(2.05618020776569710e-01, -9.83640319434999216e-02));
        if (perturbed)
            polygon.addVertex(Vector2D<>(1.45740200767926043e-01, -2.90653173265546738e-01));
            else
                polygon.addVertex(Vector2D<>(1.45740200767926098e-01, -2.90653173265546738e-01));
        polygon.addVertex(Vector2D<>(7.34225154431480675e-02, -3.75276534368993664e-01));
        if (perturbed)
            polygon.addVertex(Vector2D<>(2.59786816870647998e-17, -4.00000000000000022e-01));
        else
            polygon.addVertex(Vector2D<>(2.59786816870648029e-17, -4.00000000000000022e-01));

        const std::vector<std::vector<std::size_t> > dec = PolygonUtil::convexDecompositionIndexed(polygon);
        EXPECT_EQ(std::size_t(8),dec.size());
        double subPolyAreaTotal = 0;
        for (std::size_t i = 0; i < dec.size(); i++) {
            Polygon<Vector2D<> > p;
            for (std::size_t k = 0; k < dec[i].size(); k++)
                p.addVertex(polygon.getVertex(dec[i][k]));
            const double area = PolygonUtil::area(p);
            EXPECT_GT(area,0);
            subPolyAreaTotal += area;
        }
        ASSERT_GE(dec.size(),std::size_t(8));
        EXPECT_EQ(std::size_t(7),dec[0].size());
        EXPECT_EQ(std::size_t(5),dec[1].size());
        EXPECT_EQ(std::size_t(7),dec[2].size());
        EXPECT_EQ(std::size_t(3),dec[3].size());
        EXPECT_EQ(std::size_t(3),dec[4].size());
        EXPECT_EQ(std::size_t(3),dec[5].size());
        EXPECT_EQ(std::size_t(3),dec[6].size());
        EXPECT_EQ(std::size_t(3),dec[7].size());
        EXPECT_DOUBLE_EQ(PolygonUtil::area(polygon),subPolyAreaTotal);
        EXPECT_DOUBLE_EQ(0.582204108543769,subPolyAreaTotal);
    }
}

TEST(PolygonUtil, ConvexDecomposition2D) {
    {
        Polygon<Vector2D<> > polygon;
        polygon.addVertex(Vector2D<>(-0.05, 0.1));
        polygon.addVertex(Vector2D<>(0.2, 0.2));
        polygon.addVertex(Vector2D<>(0.35, 0.07));
        polygon.addVertex(Vector2D<>(0.5, 0.23));
        polygon.addVertex(Vector2D<>(0.7, 0.26));
        polygon.addVertex(Vector2D<>(0.7, 0.035));
        polygon.addVertex(Vector2D<>(0.4, -0.05));
        polygon.addVertex(Vector2D<>(0.55, -0.15));
        polygon.addVertex(Vector2D<>(0.68, -0.17));
        polygon.addVertex(Vector2D<>(0.45, -0.4));
        polygon.addVertex(Vector2D<>(0.275, -0.2));
        polygon.addVertex(Vector2D<>(0, -0.16));

        const std::vector<Polygon<Vector2D<> > > dec = PolygonUtil::convexDecomposition(polygon);
        EXPECT_EQ(std::size_t(4),dec.size());
        double subPolyAreaTotal = 0;
        for (std::size_t i = 0; i < dec.size(); i++) {
            const Polygon<Vector2D<> >& p = dec[i];
            const double area = PolygonUtil::area(p);
            EXPECT_LT(area,0);
            subPolyAreaTotal += area;
        }
        EXPECT_DOUBLE_EQ(PolygonUtil::area(polygon),subPolyAreaTotal);
        EXPECT_DOUBLE_EQ(-0.26475,subPolyAreaTotal);
    }

    // Test polygon where multiple consecutive vertices lies on the same line.
    SCOPED_TRACE("Test without perturbation.");
    testPerturbation(false);
    SCOPED_TRACE("Test with perturbation.");
    testPerturbation(true);
}

TEST(PolygonUtil, Area) {
	Polygon<Vector2D<> > polygonCW;
	polygonCW.addVertex(Vector2D<>(0.55, -0.15));
	polygonCW.addVertex(Vector2D<>(0.68, -0.17));
	polygonCW.addVertex(Vector2D<>(0.45, -0.4));
	polygonCW.addVertex(Vector2D<>(0.275, -0.2));

	Polygon<Vector2D<> > polygonCCW;
	polygonCCW.addVertex(Vector2D<>(0.275, -0.2));
	polygonCCW.addVertex(Vector2D<>(0.45, -0.4));
	polygonCCW.addVertex(Vector2D<>(0.68, -0.17));
	polygonCCW.addVertex(Vector2D<>(0.55, -0.15));

	EXPECT_DOUBLE_EQ(-0.049125,PolygonUtil::area(polygonCW));
	EXPECT_DOUBLE_EQ(0.049125,PolygonUtil::area(polygonCCW));
}

TEST(PolygonUtil, IsInside) {
	Polygon<Vector2D<> > polygon;
	polygon.addVertex(Vector2D<>(0.55, -0.15));
	polygon.addVertex(Vector2D<>(0.68, -0.17));
	polygon.addVertex(Vector2D<>(0.45, -0.4));
	polygon.addVertex(Vector2D<>(0.275, -0.2));

	EXPECT_TRUE(PolygonUtil::isInsideConvex(Vector2D<>(0.4,-0.3),polygon,std::numeric_limits<double>::epsilon()*5));
	EXPECT_FALSE(PolygonUtil::isInsideConvex(Vector2D<>(0.29,-0.35),polygon,std::numeric_limits<double>::epsilon()*5));
	EXPECT_FALSE(PolygonUtil::isInsideConvex(Vector2D<>(0.45,-0.4),polygon,std::numeric_limits<double>::epsilon()*5));
}
