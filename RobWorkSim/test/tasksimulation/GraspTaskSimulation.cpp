/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rw/rw.hpp>
#include <rwlibs/task.hpp>
#include <rwlibs/task/GraspTask.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsim/simulator/GraspTaskSimulator.hpp>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;

#include "../TestSuiteConfig.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

using namespace rwsim::loaders;
using namespace rwsim::simulator;
using namespace rwsim::dynamics;

BOOST_AUTO_TEST_CASE( SimpleParallelGraspAndHoldStabilityTest )
{

}

BOOST_AUTO_TEST_CASE( ComplexParallelGraspAndHoldStabilityTest )
{

}

BOOST_AUTO_TEST_CASE( SimpleDexterousGraspAndHoldStabilityTest )
{

}

BOOST_AUTO_TEST_CASE( ComplexDexterousGraspAndHoldStabilityTest )
{

}


BOOST_AUTO_TEST_CASE( GraspTaskSimulatorTest )
{
	std::string dwc_file = testFilePath() + "/scene/grasping/pg70_box.dwc.xml";
	std::string grasptask_file = testFilePath() + "/scene/grasping/grasptask_pg70_box.rwtask.xml";

    // add loading tests here
    DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load(dwc_file);
    State initState = dwc->getWorkcell()->getDefaultState();
    WorkCell::Ptr wc = dwc->getWorkcell();

    // create GraspTaskSimulator
    GraspTaskSimulator::Ptr graspSim = ownedPtr( new GraspTaskSimulator(dwc, 1) );

    GraspTask::Ptr grasptask = GraspTask::load( grasptask_file );

    graspSim->load( grasptask );
    graspSim->startSimulation(initState);
    Timer timeoutTimer;
    do{
        TimerUtil::sleepMs(100);
        if(timeoutTimer.getTimeSec()>50){
        	BOOST_CHECK_MESSAGE(timeoutTimer.getTimeSec()<50, "Tasks where not simulated fast enough! Assumed errors in simulation.");
        	break;
        }
    } while(graspSim->isRunning());

    GraspTask::Ptr grasptask_result = graspSim->getResult();

    // check stuff in results

}