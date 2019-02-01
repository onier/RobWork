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

#include <gtest/gtest.h>

#include <rw/geometry/Delaunay.hpp>

using namespace rw::geometry;
using rw::math::Vector2D;

TEST(DelaunayTest, Rectangle) {
    // This tests cocircular points (which has multiple solutions)
    static const std::vector<Vector2D<> > verticesSquare = {
            {0.005, -0.0225},
            {-0.005, -0.0225},
            {-0.005, 0.0225},
            {0.005, 0.0225}
    };
    IndexedTriMesh<>::Ptr mesh;
    ASSERT_NO_THROW(mesh = Delaunay::triangulate(verticesSquare));
    ASSERT_EQ(std::size_t(2), mesh->getSize());
    double area = 0;
    for(std::size_t i = 0; i < mesh->getSize(); i++) {
        const Triangle<> tri = mesh->getTriangle(i);
        area += tri.calcArea();
    }
    EXPECT_DOUBLE_EQ(4.5e-4,area);
}

TEST(DelaunayTest, 4Points) {
    // This tests non-cocircular points
    static const std::vector<Vector2D<> > vertices = {
            {0.005, -0.7225},
            {-0.005, -0.0225},
            {-0.005, 0.0225},
            {0.005, 0.0225}
    };
    IndexedTriMesh<>::Ptr mesh;
    ASSERT_NO_THROW(mesh = Delaunay::triangulate(vertices));
    ASSERT_EQ(std::size_t(2), mesh->getSize());
    double area = 0;
    for(std::size_t i = 0; i < mesh->getSize(); i++) {
        const Triangle<> tri = mesh->getTriangle(i);
        area += tri.calcArea();
    }
    EXPECT_DOUBLE_EQ(0.00395,area);
}
