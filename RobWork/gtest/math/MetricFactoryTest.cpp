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

#include <rw/math/MetricFactory.hpp>
#include <rw/math/Vector2D.hpp>

using namespace rw::math;

TEST(MetricFactory, Q) {
    Metric<Q>::Ptr q =
        MetricFactory::makeWeightedManhattan(Q());

    Metric<Vector2D<> >::Ptr v2 =
        MetricFactory::makeWeightedManhattan(Vector2D<>());

    Metric<Vector3D<> >::Ptr v3 =
        MetricFactory::makeWeightedManhattan(Vector3D<>());

    Metric<std::vector<double> >::Ptr sv =
        MetricFactory::makeWeightedManhattan(
            std::vector<double>());

    Metric<boost::numeric::ublas::vector<double> >::Ptr bv =
        MetricFactory::makeWeightedManhattan(
            boost::numeric::ublas::vector<double>());
}
