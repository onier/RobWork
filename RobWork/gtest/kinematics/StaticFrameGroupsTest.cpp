/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics.hpp>
#include <rw/models/WorkCell.hpp>
#include <boost/foreach.hpp>
#include <iostream>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::common;

namespace {
void addTestFrames(WorkCell& world) {
	MovableFrame* frame1 = new MovableFrame("Frame1");
	MovableFrame* frame2 = new MovableFrame("Frame2");
	MovableFrame* frame3 = new MovableFrame("Frame3");
	FixedFrame* frame4 = new FixedFrame("Frame4",Transform3D<double>());
	FixedFrame* frame5 = new FixedFrame("Frame5",Transform3D<double>());
	MovableFrame* frame6 = new MovableFrame("Frame6");
	MovableFrame* frame7 = new MovableFrame("Frame7");
	MovableFrame* frame8 = new MovableFrame("Frame8");
	MovableFrame* frame9 = new MovableFrame("Frame9");
	MovableFrame* frame10 = new MovableFrame("Frame10");
	MovableFrame* frame11 = new MovableFrame("Frame11");
	FixedFrame* frame12 = new FixedFrame("Frame12",Transform3D<double>());

	world.addFrame(frame1);
	world.addFrame(frame3,frame1);
	world.addFrame(frame4,frame1);
	world.addDAF(frame8,frame1);
	world.addDAF(frame7,frame4);

	world.addFrame(frame2);
	world.addFrame(frame5,frame2);
	world.addFrame(frame6,frame5);
	world.addFrame(frame9,frame6);
	world.addDAF(frame10,frame9);
	world.addDAF(frame11,frame9);

	world.addFrame(frame12);
}
}

TEST(Kinematics, getStaticFrameGroups) {
	WorkCell world("The World");
	addTestFrames(world);

	std::vector<bool> frameInGroup(13,false);

	Frame* const root = world.getWorldFrame();
	std::vector<FrameList> staticGroups = Kinematics::getStaticFrameGroups(root,world.getDefaultState());
	EXPECT_EQ(10,staticGroups.size());
	BOOST_FOREACH(FrameList& list, staticGroups) {
		BOOST_FOREACH(const Frame* frame, list) {
			if (frame->getName() == "WORLD") {
				EXPECT_FALSE(frameInGroup[0]);
				frameInGroup[0] = true;
			} else {
				const int id = std::stoi(frame->getName().substr(5));
				EXPECT_FALSE(frameInGroup[id]);
				frameInGroup[id] = true;
			}
		}
	}
	for(std::size_t i = 0; i < frameInGroup.size(); i++) {
		EXPECT_TRUE(frameInGroup[i]);
	}
}

TEST(Kinematics, getStaticFrameGroupsConst) {
	WorkCell world("The World");
	addTestFrames(world);

	std::vector<bool> frameInGroup(13,false);

	const Frame* const root = world.getWorldFrame();
	std::vector<ConstFrameList> staticGroups = Kinematics::getStaticFrameGroups(root,world.getDefaultState());
	EXPECT_EQ(10,staticGroups.size());
	BOOST_FOREACH(ConstFrameList& list, staticGroups) {
		BOOST_FOREACH(const Frame* frame, list) {
			if (frame->getName() == "WORLD") {
				EXPECT_FALSE(frameInGroup[0]);
				frameInGroup[0] = true;
			} else {
				const int id = std::stoi(frame->getName().substr(5));
				EXPECT_FALSE(frameInGroup[id]);
				frameInGroup[id] = true;
			}
		}
	}
	for(std::size_t i = 0; i < frameInGroup.size(); i++) {
		EXPECT_TRUE(frameInGroup[i]);
	}
}
