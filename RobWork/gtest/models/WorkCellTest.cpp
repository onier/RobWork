/********************************************************************************
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
 ********************************************************************************/

#include <gtest/gtest.h>

#include <iostream>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/models/WorkCell.hpp>

#include <boost/foreach.hpp>
#include <vector>

using namespace std;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::common;

TEST(WorkCell, AddRemoveFrame) {
    const MovableFrame::Ptr frame1 = new MovableFrame("Frame1");
    MovableFrame* frame2 = new MovableFrame("Frame2");

    const boost::shared_ptr<MovableFrame> boostframe3(new MovableFrame("Frame3"));
    const MovableFrame::Ptr frame3(boostframe3);

    const MovableFrame::Ptr frame4 = new MovableFrame("Frame4");
    MovableFrame* const frame5 = new MovableFrame("Frame5");

    const boost::shared_ptr<MovableFrame> boostframe6(new MovableFrame("Frame6"));
    const MovableFrame::Ptr frame6(boostframe6);

    WorkCell* world;

    world = new WorkCell("The World");
    ASSERT_THROW(world->addFrame(frame1), rw::common::Exception);
    world->addFrame(frame2);
    EXPECT_NO_THROW(world->addFrame(frame3));
    EXPECT_THROW(world->addFrame(frame1,frame2), rw::common::Exception);
    delete world;

    world = new WorkCell("The World");
    frame2 = new MovableFrame("Frame2");
    world->addFrame(frame2);
    EXPECT_NO_THROW(world->addFrame(frame3,frame2));
    EXPECT_NO_THROW(world->addFrame(frame6,frame2));
    delete world;

    world = new WorkCell("The World");
    frame2 = new MovableFrame("Frame2");
    ASSERT_NO_THROW(world->addFrame(frame3));
    ASSERT_THROW(world->addFrame(frame2,frame3), rw::common::Exception);
    EXPECT_NO_THROW(world->addFrame(frame6,frame3));
    delete world;
    delete frame2;

    world = new WorkCell("The World");
    frame2 = new MovableFrame("Frame2");
    ASSERT_THROW(world->addDAF(frame1), rw::common::Exception);
    world->addDAF(frame2);
    EXPECT_NO_THROW(world->addDAF(frame3));
    EXPECT_THROW(world->addDAF(frame1,frame2), rw::common::Exception);
    delete world;

    world = new WorkCell("The World");
    frame2 = new MovableFrame("Frame2");
    world->addFrame(frame2);
    EXPECT_NO_THROW(world->addDAF(frame3,frame2));
    EXPECT_NO_THROW(world->addDAF(frame6,frame2));
    delete world;

    world = new WorkCell("The World");
    frame2 = new MovableFrame("Frame2");
    ASSERT_NO_THROW(world->addFrame(frame3));
    EXPECT_THROW(world->addDAF(frame2,frame3), rw::common::Exception);
    EXPECT_NO_THROW(world->addDAF(frame6,frame3));
    delete world;
    delete frame2;

    world = new WorkCell("The World");
    frame2 = new MovableFrame("Frame2");
    world->addFrame(frame2);
    ASSERT_NO_THROW(world->addFrame(frame3));
    world->addFrame(frame5);
    ASSERT_NO_THROW(world->addFrame(frame6));
    world->remove(frame2); // deletes frame2
    world->remove(frame3);
    EXPECT_EQ(-1, frame3->getID());
    world->remove(frame5); // deletes frame5
    world->remove(frame6);
    EXPECT_EQ(-1, frame6->getID());
    ASSERT_NO_THROW(world->addFrame(frame3));
    ASSERT_NO_THROW(world->addFrame(frame6));
    delete world;

    delete frame1.get();
    delete frame4.get();
}
