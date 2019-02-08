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

#include <rw/common/ConcatVectorIterator.hpp>
#include <rw/common/VectorIterator.hpp>

using namespace rw::common;

namespace {
    std::string str1 = "abc";
    std::string str2 = "de";
}

TEST(Iterator, ConcatVectorIterator) {
    std::vector<char*> curr = {&str1[0], &str1[1], &str1[2]};
    std::vector<char*> next = {&str2[0], &str2[1]};

    ConcatVectorIterator<char> begin(&curr, curr.begin(), &next);
    ConcatVectorIterator<char> end(&next, next.end(), NULL);

    ConstConcatVectorIterator<char> const_begin(begin);

    ConstConcatVectorIterator<char> f = begin;
    EXPECT_EQ('a', *f);
    EXPECT_EQ('b', *++f);

    EXPECT_EQ('a', *const_begin++);
    EXPECT_EQ('b', *const_begin);

    char& x = *begin;
    EXPECT_EQ('a', x);

    const char& c = *const_begin;
    EXPECT_EQ('b', c);

    EXPECT_EQ('c', *++const_begin);
    EXPECT_EQ('c', *const_begin++);
    EXPECT_EQ('d', *const_begin);
    const_begin++;
    EXPECT_EQ('e', *const_begin++);
    EXPECT_TRUE(const_begin == end);
}

TEST(Iterator, VectorIterator) {
    std::vector<char*> vals = {&str1[0], &str1[1], &str1[2]};
    VectorIterator<char> it(vals.begin());
    EXPECT_EQ('a', *it);
    EXPECT_EQ('a', *it++);
    EXPECT_EQ('b', *it);
    EXPECT_EQ('c', *++it);
    it++;
    EXPECT_TRUE(it == VectorIterator<char>(vals.end()));
}
