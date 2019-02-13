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

#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>

using rw::common::ownedPtr;
using namespace rw::math;
using namespace rw::pathplanning;
using rw::trajectory::QPath;
using rwlibs::pathoptimization::PathLengthOptimizer;

namespace {
    class TestQConstraint: public QConstraint
    {
        protected:
            virtual bool doInCollision(const Q& q) const
            {
                // not checked in pathPruning algorithm
                return false;
            }

            virtual void doSetLog(rw::common::Log::Ptr log) {}
    };

    class TestQEdgeConstraint: public QEdgeConstraint
    {
        protected:
            virtual bool doInCollision(const Q& start, const Q& end) const
            {
                // Make a collision between q1 and q3
                if (start[0] < 2.5 && start[1] < 0.5 && end[0] < 2.5 && end[1] < 0.5)
                    return true;
                else
                    return false;
            }
    };
}

TEST(PathLengthOptimizer, pathPruning) {
    static const Q q1(2,0.,0.);
    static const Q q2(2,1.,1.);
    static const Q q3(2,2.,0.);
    static const Q q4(2,2.,1.);
    static const Q q5(2,2.,2.);
    const QConstraint::Ptr qConstraint = ownedPtr(new TestQConstraint());
    const QEdgeConstraint::Ptr edgeConstraint = ownedPtr(new TestQEdgeConstraint());
    const PlannerConstraint constraint(qConstraint, edgeConstraint);
    const QMetric::CPtr metric;
    const PathLengthOptimizer optimizer(constraint, metric);
    QPath path;
    path.push_back(q1);
    path.push_back(q2);
    path.push_back(q3);
    path.push_back(q4);
    path.push_back(q5);
    const QPath pathAfter = optimizer.pathPruning(path);
    ASSERT_EQ((std::size_t)2, pathAfter.size());
    EXPECT_TRUE(pathAfter[0] == q1);
    EXPECT_TRUE(pathAfter[1] == q5);
}
