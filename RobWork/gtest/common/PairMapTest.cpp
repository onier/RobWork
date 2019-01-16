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

#include <rw/common/PairMap.hpp>

using rw::common::PairMap;

TEST(PairMapTest, IntPair) {
    PairMap<int,int> pmap(123);
    const PairMap<int,int>& cmap = pmap;

    // Test while empty
    EXPECT_FALSE(pmap.has(std::make_pair(-45,10)));
    EXPECT_FALSE(pmap.has(-45,10));
    EXPECT_EQ(123,cmap[std::make_pair(-45,10)]);
    EXPECT_EQ(123,cmap(-45,10));
    EXPECT_EQ(std::size_t(0),pmap.size());
    EXPECT_GT(pmap.max_size(),std::size_t(10000));
    ASSERT_GT(pmap.max_size(),std::size_t(4));
    EXPECT_TRUE(pmap.empty());

    // Insert a few pairs
    pmap.insert(std::make_pair(12,10),-1);
    pmap.insert(std::make_pair(12,17),-2);
    EXPECT_EQ(std::size_t(2),pmap.size());
    EXPECT_EQ(123,pmap[std::make_pair(-7,9)]);
    EXPECT_EQ(std::size_t(3),pmap.size());
    pmap[std::make_pair(9,-7)] = -123;
    EXPECT_EQ(std::size_t(3),pmap.size());
    EXPECT_EQ(123,pmap(-127,1));
    EXPECT_EQ(std::size_t(4),pmap.size());
    pmap(-127,1) = 43;
    EXPECT_EQ(std::size_t(4),pmap.size());

    // Test the values
    EXPECT_EQ(-1,pmap[std::make_pair(12,10)]);
    EXPECT_EQ(-1,pmap[std::make_pair(10,12)]);
    EXPECT_EQ(-1,cmap[std::make_pair(10,12)]);
    EXPECT_EQ(-2,pmap(12,17));
    EXPECT_EQ(-2,pmap(17,12));
    EXPECT_EQ(-123,pmap[std::make_pair(-7,9)]);
    EXPECT_EQ(43,pmap(1,-127));
    EXPECT_EQ(std::size_t(4),pmap.size());

    // Clear the map
    pmap.clear();
    EXPECT_FALSE(pmap.has(-7,9));
    EXPECT_EQ(123,cmap[std::make_pair(10,12)]);
    EXPECT_EQ(std::size_t(0),pmap.size());
}
