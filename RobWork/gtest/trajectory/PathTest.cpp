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

#include <rw/math/Random.hpp>
#include <rw/trajectory/Path.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include <sstream>

using namespace rw::math;
using rw::trajectory::QPath;

TEST(BoostSerialization, Path) {
    std::stringstream stream;

    QPath path(5);

    for(std::size_t i = 0 ; i < path.size(); ++i) {
        std::size_t size = Random::ranI(2, 15);
        path[i] = Q(size);

        for(std::size_t k = 0; k < size; ++k) {
            path[i][k] = Random::ran(-10, 10);
        }
    }

    boost::archive::text_oarchive oa(stream);
    oa << path;

    // Use boost to load class data
    boost::archive::text_iarchive ia(stream);
    QPath loadedpath;
    ia >> loadedpath;

    EXPECT_EQ(path.size(), loadedpath.size());
    for(std::size_t i = 0; i < std::min(path.size(),loadedpath.size()); ++i) {
        EXPECT_EQ(path[i], loadedpath[i]);
    }
}
