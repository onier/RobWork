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

#include <rw/geometry/analytic/ImplicitTorus.hpp>

using rw::geometry::ImplicitTorus;
using rw::math::Vector3D;

TEST(ImplicitTorus, CircularTorusCircularTube) {
    static const double R = 2.;
    static const double r = 0.5;
    const ImplicitTorus torus(R, r);

    EXPECT_DOUBLE_EQ(0.,torus(Vector3D<>(R,0,r)));
    EXPECT_DOUBLE_EQ(0.,torus(Vector3D<>(0,R,r)));
    EXPECT_LT(torus(Vector3D<>(R,0,0)),0.);
    EXPECT_GT(torus(Vector3D<>(R,0,2.*r)),0.);

    EXPECT_DOUBLE_EQ(1.,torus.normal(Vector3D<>(R,0,r))[2]);
    EXPECT_DOUBLE_EQ(-1.,torus.normal(Vector3D<>(R-r,0,0))[0]);
    EXPECT_DOUBLE_EQ(1.,torus.normal(Vector3D<>(R+r,0,0))[0]);
    EXPECT_DOUBLE_EQ(1.,torus.normal(Vector3D<>(0,R,r))[2]);
    EXPECT_DOUBLE_EQ(-1.,torus.normal(Vector3D<>(0,R-r,0))[1]);
    EXPECT_DOUBLE_EQ(1.,torus.normal(Vector3D<>(0,R+r,0))[1]);
}

TEST(ImplicitTorus, CircularTorusEllipticTube) {
    static const double R = 2.;
    static const double r1 = 0.5;
    static const double r2 = 1.5;
    const ImplicitTorus torus(R, R, r1, r2);

    EXPECT_DOUBLE_EQ(0.,torus(Vector3D<>(R,0,r2)));
    EXPECT_DOUBLE_EQ(0.,torus(Vector3D<>(0,R,r2)));
    EXPECT_LT(torus(Vector3D<>(R,0,0)),0.);
    EXPECT_GT(torus(Vector3D<>(R,0,2.*r2)),0.);

    EXPECT_DOUBLE_EQ(1.,torus.normal(Vector3D<>(R,0,r2))[2]);
    EXPECT_DOUBLE_EQ(-1.,torus.normal(Vector3D<>(R-r1,0,0))[0]);
    EXPECT_DOUBLE_EQ(1.,torus.normal(Vector3D<>(R+r1,0,0))[0]);
    EXPECT_DOUBLE_EQ(1.,torus.normal(Vector3D<>(0,R,r2))[2]);
    EXPECT_DOUBLE_EQ(-1.,torus.normal(Vector3D<>(0,R-r1,0))[1]);
    EXPECT_DOUBLE_EQ(1.,torus.normal(Vector3D<>(0,R+r1,0))[1]);
}

TEST(ImplicitTorus, EllipticTorusCircularTube) {
    static const double R1 = 2.;
    static const double R2 = 3.;
    static const double r = 0.5;
    const ImplicitTorus torus(R1, R2, r, r);

    EXPECT_DOUBLE_EQ(0.,torus(Vector3D<>(R1,0,r)));
    EXPECT_DOUBLE_EQ(0.,torus(Vector3D<>(0,R2,r)));
    EXPECT_LT(torus(Vector3D<>(R1,0,0)),0.);
    EXPECT_GT(torus(Vector3D<>(R1,0,2.*r)),0.);

    EXPECT_DOUBLE_EQ(1.,torus.normal(Vector3D<>(R1,0,r))[2]);
    EXPECT_DOUBLE_EQ(-1.,torus.normal(Vector3D<>(R1-r,0,0))[0]);
    EXPECT_DOUBLE_EQ(1.,torus.normal(Vector3D<>(R1+r,0,0))[0]);
    EXPECT_DOUBLE_EQ(1.,torus.normal(Vector3D<>(0,R2,r))[2]);
    EXPECT_DOUBLE_EQ(-1.,torus.normal(Vector3D<>(0,R2-r,0))[1]);
    EXPECT_DOUBLE_EQ(1.,torus.normal(Vector3D<>(0,R2+r,0))[1]);
}

TEST(ImplicitTorus, EllipticTorusEllipticTube) {
    static const double R1 = 2.;
    static const double R2 = 3.;
    static const double r1 = 0.5;
    static const double r2 = 1.5;
    const ImplicitTorus torus(R1, R2, r1, r2);

    EXPECT_DOUBLE_EQ(0.,torus(Vector3D<>(R1,0,r2)));
    EXPECT_DOUBLE_EQ(0.,torus(Vector3D<>(0,R2,r2)));
    EXPECT_LT(torus(Vector3D<>(R1,0,0)),0.);
    EXPECT_GT(torus(Vector3D<>(R1,0,2.*r2)),0.);

    EXPECT_DOUBLE_EQ(1.,torus.normal(Vector3D<>(R1,0,r2))[2]);
    EXPECT_DOUBLE_EQ(-1.,torus.normal(Vector3D<>(R1-r1,0,0))[0]);
    EXPECT_DOUBLE_EQ(1.,torus.normal(Vector3D<>(R1+r1,0,0))[0]);
    EXPECT_DOUBLE_EQ(1.,torus.normal(Vector3D<>(0,R2,r2))[2]);
    EXPECT_DOUBLE_EQ(-1.,torus.normal(Vector3D<>(0,R2-r1,0))[1]);
    EXPECT_DOUBLE_EQ(1.,torus.normal(Vector3D<>(0,R2+r1,0))[1]);
}
