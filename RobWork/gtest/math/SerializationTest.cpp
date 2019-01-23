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

#include <rw/math/Q.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include <sstream>

using namespace rw::math;

namespace {
    class MyClass {
        public:
            Q myQ;
            Vector2D<double> myVec2D;
            Vector2D<float> myVec2Df;
            Vector3D<double> myVec3D;
            Vector3D<float> myVec3Df;
            Rotation3D<double> myR;
            Rotation3D<float> myRf;
            Transform3D<double> myT;
            Transform3D<float> myTf;

        private:
            friend class boost::serialization::access;

            template<class Archive>
            void serialize(Archive & ar, const unsigned int version)
            {
                ar & myQ;
                ar & myVec2D;
                ar & myVec2Df;
                ar & myVec3D;
                ar & myVec3Df;
                ar & myR;
                ar & myRf;
                ar & myT;
                ar & myTf;
            }
    };
}

TEST(BoostSerialization, Q) {
    static const Q q(3, 0.11, 0.12, 0.13);
    static const Vector2D<double> vec2(0.1, 0.2);
    static const Vector2D<float> vec2f(0.3f, 0.4f);
    static const Vector3D<double> vec3(0.5, 0.6, 0.7);
    static const Vector3D<float> vec3f(0.8f, 0.9f, 1.0f);
    static const Rotation3D<double> R(1.1, 1.2, 1.3, 2.1, 2.2, 2.3, 3.1, 3.2, 3.3);
    static const Rotation3D<float> Rf(4.1f, 4.2f, 4.3f, 5.1f, 5.2f, 5.3f, 6.1f, 6.2f, 6.3f);
    static const Transform3D<double> T(vec3, R);
    static const Transform3D<float> Tf(vec3f, Rf);
    std::stringstream stream;

    //save
    {
        MyClass myClassObj;
        myClassObj.myQ = q;
        myClassObj.myVec2D = vec2;
        myClassObj.myVec2Df = vec2f;
        myClassObj.myVec3D = vec3;
        myClassObj.myVec3Df = vec3f;
        myClassObj.myR = R;
        myClassObj.myRf = Rf;
        myClassObj.myT = T;
        myClassObj.myTf = Tf;

        boost::archive::text_oarchive oa(stream);
        oa << myClassObj;
    }

    // load
    {
        MyClass myClassObj;
        boost::archive::text_iarchive ia(stream);
        ia >> myClassObj;
        EXPECT_EQ(q, myClassObj.myQ);
        EXPECT_EQ(vec2, myClassObj.myVec2D);
        EXPECT_EQ(vec2f, myClassObj.myVec2Df);
        EXPECT_EQ(vec3, myClassObj.myVec3D);
        EXPECT_EQ(vec3f, myClassObj.myVec3Df);
        EXPECT_EQ(R, myClassObj.myR);
        EXPECT_EQ(Rf, myClassObj.myRf);
        EXPECT_EQ(T, myClassObj.myT);
        EXPECT_EQ(Tf, myClassObj.myTf);
        EXPECT_TRUE(myClassObj.myQ == q);
        EXPECT_TRUE(myClassObj.myVec2D == vec2);
        EXPECT_TRUE(myClassObj.myVec2Df == vec2f);
        EXPECT_TRUE(myClassObj.myVec3D == vec3);
        EXPECT_TRUE(myClassObj.myVec3Df == vec3f);
        EXPECT_TRUE(myClassObj.myR == R);
        EXPECT_TRUE(myClassObj.myRf == Rf);
        EXPECT_TRUE(myClassObj.myT == T);
        EXPECT_TRUE(myClassObj.myTf == Tf);
    }
}
