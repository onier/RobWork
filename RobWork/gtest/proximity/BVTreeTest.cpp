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

#include <rw/geometry/analytic/BREP.hpp>
#include <rw/loaders/GeometryFactory.hpp>
#include <rw/proximity/rwstrategy/BVTreeFactory.hpp>

using namespace rw::geometry;
using rw::loaders::GeometryFactory;
using namespace rw::proximity;

TEST(BVTreeFactory, Quadratics) {
    typedef BinaryBVTree<OBB<double>, GenericFace> BinaryTree;

    const Geometry::Ptr geomA = GeometryFactory::getGeometry("#Custom QuadraticTestObjectA ");
    const Geometry::Ptr geomB = GeometryFactory::getGeometry("#Custom QuadraticTestObjectB ");
    ASSERT_FALSE(geomB.isNull());
    ASSERT_FALSE(geomA.isNull());
    const BREP::Ptr objectA = geomA->getGeometryData().cast<BREP>();
    const BREP::Ptr objectB = geomB->getGeometryData().cast<BREP>();
    ASSERT_FALSE(objectA.isNull());
    ASSERT_FALSE(objectB.isNull());

    objectA->scale(0.1);
    objectB->scale(0.1);
    objectA->setMeshResolution(10);
    objectB->setMeshResolution(25);

    const Shell::CPtr shellA = objectA->shellProxy();
    const Shell::CPtr shellB = objectB->shellProxy();

    BVTreeFactory factory;
    const BinaryTree* const treeA =
            factory.makeTopDownOBBTreeMedian<BinaryTree>(shellA, 1);
    const BinaryTree* const treeB =
            factory.makeTopDownOBBTreeMedian<BinaryTree>(shellB, 1);

    // Test number of leafs
    EXPECT_EQ(static_cast<int>(shellA->size()), treeA->countNodes().second);
    EXPECT_EQ(static_cast<int>(shellB->size()), treeB->countNodes().second);

    // Test number of non-leaf nodes
    EXPECT_EQ(7, treeA->countNodes().first);
    EXPECT_EQ(5, treeB->countNodes().first);

    // Iterate through A
    {
        BinaryTree::NodeIterator it = treeA->getRootIterator();
        ASSERT_TRUE(it.hasLeft());
        ASSERT_TRUE(it.hasRight());
        BinaryTree::NodeIterator itLeft = it.left();
        BinaryTree::NodeIterator itRight = it.right();
        // in first division OBVs increase in volume
        //EXPECT_LT(itLeft.bv().calcVolume(), it.bv().calcVolume());
        //EXPECT_LT(itRight.bv().calcVolume(), it.bv().calcVolume());
        {
            BinaryTree::NodeIterator it = itLeft;
            ASSERT_TRUE(it.hasLeft());
            ASSERT_TRUE(it.hasRight());
            BinaryTree::NodeIterator itLeft = it.left();
            BinaryTree::NodeIterator itRight = it.right();
            EXPECT_LT(itLeft.bv().calcVolume(), it.bv().calcVolume());
            EXPECT_LT(itRight.bv().calcVolume(), it.bv().calcVolume());
            {
                BinaryTree::NodeIterator it = itLeft;
                ASSERT_TRUE(it.hasLeft());
                ASSERT_TRUE(it.hasRight());
                BinaryTree::NodeIterator itLeft = it.left();
                BinaryTree::NodeIterator itRight = it.right();
                EXPECT_LT(itLeft.bv().calcVolume(), it.bv().calcVolume());
                EXPECT_LT(itRight.bv().calcVolume(), it.bv().calcVolume());
                EXPECT_EQ(0, itLeft.bv().calcVolume());
                EXPECT_EQ(std::size_t(6), itLeft.primitiveIdx());
                EXPECT_EQ(std::size_t(0), itRight.primitiveIdx());
            }
            {
                BinaryTree::NodeIterator it = itRight;
                ASSERT_TRUE(it.hasLeft());
                ASSERT_TRUE(it.hasRight());
                BinaryTree::NodeIterator itLeft = it.left();
                BinaryTree::NodeIterator itRight = it.right();
                EXPECT_LT(itLeft.bv().calcVolume(), it.bv().calcVolume());
                EXPECT_LT(itRight.bv().calcVolume(), it.bv().calcVolume());
                EXPECT_LT(itLeft.bv().calcVolume(), 1e-15);
                EXPECT_LT(itRight.bv().calcVolume(), 1e-15);
                EXPECT_EQ(std::size_t(3), itLeft.primitiveIdx());
                EXPECT_EQ(std::size_t(4), itRight.primitiveIdx());
            }
        }
        {
            BinaryTree::NodeIterator it = itRight;
            ASSERT_TRUE(it.hasLeft());
            ASSERT_TRUE(it.hasRight());
            BinaryTree::NodeIterator itLeft = it.left();
            BinaryTree::NodeIterator itRight = it.right();
            EXPECT_LT(itLeft.bv().calcVolume(), it.bv().calcVolume());
            EXPECT_LT(itRight.bv().calcVolume(), it.bv().calcVolume());
            {
                BinaryTree::NodeIterator it = itLeft;
                ASSERT_TRUE(it.hasLeft());
                ASSERT_TRUE(it.hasRight());
                BinaryTree::NodeIterator itLeft = it.left();
                BinaryTree::NodeIterator itRight = it.right();
                EXPECT_LT(itLeft.bv().calcVolume(), it.bv().calcVolume());
                EXPECT_LT(itRight.bv().calcVolume(), it.bv().calcVolume());
                EXPECT_LT(itRight.bv().calcVolume(), 1e-15);
                EXPECT_EQ(std::size_t(1), itLeft.primitiveIdx());
                EXPECT_EQ(std::size_t(7), itRight.primitiveIdx());
            }
            {
                BinaryTree::NodeIterator it = itRight;
                ASSERT_TRUE(it.hasLeft());
                ASSERT_TRUE(it.hasRight());
                BinaryTree::NodeIterator itLeft = it.left();
                BinaryTree::NodeIterator itRight = it.right();
                EXPECT_LT(itLeft.bv().calcVolume(), it.bv().calcVolume());
                EXPECT_LT(itRight.bv().calcVolume(), it.bv().calcVolume());
                EXPECT_EQ(0, itLeft.bv().calcVolume());
                EXPECT_LT(itRight.bv().calcVolume(), 1e-15);
                EXPECT_EQ(std::size_t(5), itLeft.primitiveIdx());
                EXPECT_EQ(std::size_t(2), itRight.primitiveIdx());
            }
        }
    }

    // Iterate through B
    {
        BinaryTree::NodeIterator it = treeB->getRootIterator();
        ASSERT_TRUE(it.hasLeft());
        ASSERT_TRUE(it.hasRight());
        BinaryTree::NodeIterator itLeft = it.left();
        BinaryTree::NodeIterator itRight = it.right();
        // in first division OBVs increase in volume
        EXPECT_LT(itLeft.bv().calcVolume(), it.bv().calcVolume());
        EXPECT_LT(itRight.bv().calcVolume(), it.bv().calcVolume());
        {
            BinaryTree::NodeIterator it = itLeft;
            ASSERT_TRUE(it.hasLeft());
            ASSERT_TRUE(it.hasRight());
            BinaryTree::NodeIterator itLeft = it.left();
            BinaryTree::NodeIterator itRight = it.right();
            EXPECT_LT(itLeft.bv().calcVolume(), it.bv().calcVolume());
            EXPECT_LT(itRight.bv().calcVolume(), it.bv().calcVolume());
            EXPECT_LT(itLeft.bv().calcVolume(), 1e-15);
            EXPECT_EQ(std::size_t(5), itLeft.primitiveIdx());
            {
                BinaryTree::NodeIterator it = itRight;
                ASSERT_TRUE(it.hasLeft());
                ASSERT_TRUE(it.hasRight());
                BinaryTree::NodeIterator itLeft = it.left();
                BinaryTree::NodeIterator itRight = it.right();
                //EXPECT_LT(itLeft.bv().calcVolume(), it.bv().calcVolume());
                EXPECT_LT(itRight.bv().calcVolume(), it.bv().calcVolume());
                EXPECT_LT(itRight.bv().calcVolume(), 1e-15);
                EXPECT_EQ(std::size_t(2), itLeft.primitiveIdx());
                EXPECT_EQ(std::size_t(4), itRight.primitiveIdx());
            }
        }
        {
            BinaryTree::NodeIterator it = itRight;
            ASSERT_TRUE(it.hasLeft());
            ASSERT_TRUE(it.hasRight());
            BinaryTree::NodeIterator itLeft = it.left();
            BinaryTree::NodeIterator itRight = it.right();
            EXPECT_LT(itLeft.bv().calcVolume(), it.bv().calcVolume());
            //EXPECT_LT(itRight.bv().calcVolume(), it.bv().calcVolume());
            EXPECT_EQ(std::size_t(3), itLeft.primitiveIdx());
            {
                BinaryTree::NodeIterator it = itRight;
                ASSERT_TRUE(it.hasLeft());
                ASSERT_TRUE(it.hasRight());
                BinaryTree::NodeIterator itLeft = it.left();
                BinaryTree::NodeIterator itRight = it.right();
                EXPECT_LT(itLeft.bv().calcVolume(), it.bv().calcVolume());
                EXPECT_LT(itRight.bv().calcVolume(), it.bv().calcVolume());
                EXPECT_EQ(std::size_t(1), itLeft.primitiveIdx());
                EXPECT_EQ(std::size_t(0), itRight.primitiveIdx());
            }
        }
    }
}
