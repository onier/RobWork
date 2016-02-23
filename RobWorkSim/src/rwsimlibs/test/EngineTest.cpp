/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "EngineTest.hpp"
#include "FreeMotionTest.hpp"
#include "CollisionBouncingBallTest.hpp"
#include "CollisionBouncingBoxTest.hpp"
#include "CollisionBouncingCylinderTest.hpp"
#include "ConstraintRevoluteTest.hpp"
#include "BowlFeederDropTest.hpp"
#include "BallDropTest.hpp"
#include "StackingBallsTest.hpp"
#include "IntegratorRotationTest.hpp"
#include "IntegratorSpringTest.hpp"
#include "FrictionSlidingTest.hpp"
#include "FrictionStaticTest.hpp"
#include "PiHCompliantTest.hpp"
#include "TubePlaneTest.hpp"
#include "KVMDropTest.hpp"
#include "KVMNutPlaneTest.hpp"
#include "KVMTest.hpp"
#include "AssemblyPegInTube.hpp"
#include "MotorTest.hpp"
#include "TwoFingerGraspTest.hpp"

#include <rw/common/ThreadTask.hpp>
#include <rw/common/ThreadSafeVariable.hpp>

using namespace rw::common;
using namespace rw::trajectory;
using namespace rwsim::log;
using namespace rwsimlibs::test;

namespace {
class RunTask: public ThreadTask {
private:
	EngineTest* const _test;
	const std::string _engineID;
	const PropertyMap& _map;
	rw::common::Ptr<rwsim::log::SimulatorLogScope> _verbose;
	EngineTest::TestHandle::Ptr _handle;

public:
	RunTask(ThreadTask::Ptr parent, EngineTest* test, const std::string& engineID, const PropertyMap& map, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose):
		ThreadTask(parent),
		_test(test),
		_engineID(engineID),
		_map(map),
		_verbose(verbose),
		_handle(ownedPtr(new EngineTest::TestHandle()))
	{
	}

	virtual ~RunTask() {
	}

	void run() {
		try {
			_test->run(_handle, _engineID, _map, _verbose);
		} catch(const Exception& e) {
			registerFailure(e);
		}
	}

	void abort() {
		_handle->abort();
	}

	EngineTest::TestHandle::Ptr getHandle() const {
		return _handle;
	}
};
}

EngineTest::Result::Result(const std::string& name, const std::string& description):
	name(name),
	description(description)
{
}

EngineTest::TestHandle::TestHandle():
	_abort(new ThreadSafeVariable<bool>(false)),
	_cb(NULL)
{
}

EngineTest::TestHandle::~TestHandle() {
	delete _abort;
}

std::string EngineTest::TestHandle::getError() const {
	return _error;
}

TimedStatePath EngineTest::TestHandle::getTimedStatePath() const {
	return _path;
}

const std::vector<EngineTest::Result>& EngineTest::TestHandle::getResults() const {
	return _results;
}

void EngineTest::TestHandle::setError(const std::string& error) {
	_error = error;
}

void EngineTest::TestHandle::append(const TimedState& tstate) {
	_path.push_back(tstate);
}

void EngineTest::TestHandle::append(const Result& result) {
	_results.push_back(result);
}

bool EngineTest::TestHandle::isAborted() {
	return _abort->getVariable();
}

void EngineTest::TestHandle::abort() {
	_abort->setVariable(true);
}

void EngineTest::TestHandle::setTimeCallback(TimeCallback cb) {
	_cb = cb;
}

void EngineTest::TestHandle::callback(double a,bool b,bool c) {
	if (!_cb.empty())
		_cb(a,b,c);
}

EngineTest::EngineTest()
{
}

EngineTest::~EngineTest() {
}

EngineTest::TestHandle::Ptr EngineTest::runThread(const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose, ThreadTask::Ptr task) {
	if (task == NULL) {
		const TestHandle::Ptr handle = ownedPtr(new TestHandle());
		run(handle, engineID, parameters, verbose);
		return handle;
	} else {
		RunTask* const runTask = new RunTask(task, this, engineID, parameters, verbose);
		task->addSubTask(ownedPtr(runTask));
		return runTask->getHandle();
	}
}

PropertyMap::Ptr EngineTest::getDefaultParameters() const {
	return ownedPtr(new PropertyMap());
}

EngineTest::Factory::Factory():
	ExtensionPoint<EngineTest>("rwsimlibs.test.EngineTest", "EngineTest extension point.")
{
}

std::vector<std::string> EngineTest::Factory::getTests() {
    std::vector<std::string> ids;
    EngineTest::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    ids.push_back("FreeMotion");
    ids.push_back("CollisionBouncingBallTest");
    ids.push_back("CollisionBouncingBoxTest");
    ids.push_back("CollisionBouncingCylinderTest");
    ids.push_back("ConstraintRevoluteTest");
    ids.push_back("BowlFeederDrop");
    ids.push_back("BallDropTest");
    ids.push_back("StackingBallsTest");
    ids.push_back("IntegratorRotationTest");
    ids.push_back("IntegratorSpringTest");
    ids.push_back("FrictionSlidingTest");
    ids.push_back("FrictionStaticTest");
    ids.push_back("PiHCompliantTest");
    ids.push_back("TubePlaneTest");
    ids.push_back("KVMDropTest");
    ids.push_back("KVMNutPlaneTest");
    ids.push_back("KVMTest");
    ids.push_back("AssemblyPegInTube");
    ids.push_back("MotorTest");
    ids.push_back("TwoFingerGraspTest");
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
        ids.push_back( ext.getProperties().get("testID",ext.name) );
    }
    return ids;
}

bool EngineTest::Factory::hasTest(const std::string& test) {
    if(test == "FreeMotion")
        return true;
    else if(test == "CollisionBouncingBallTest")
        return true;
    else if(test == "CollisionBouncingBoxTest")
        return true;
    else if(test == "CollisionBouncingCylinderTest")
        return true;
    else if(test == "ConstraintRevoluteTest")
        return true;
    else if(test == "BowlFeederDrop")
        return true;
    else if(test == "BallDropTest")
        return true;
    else if(test == "StackingBallsTest")
        return true;
    else if(test == "IntegratorRotationTest")
        return true;
    else if(test == "IntegratorSpringTest")
        return true;
    else if(test == "FrictionSlidingTest")
        return true;
    else if(test == "FrictionStaticTest")
        return true;
    else if(test == "PiHCompliantTest")
        return true;
    else if(test == "TubePlaneTest")
        return true;
    else if(test == "KVMDropTest")
        return true;
    else if(test == "KVMNutPlaneTest")
        return true;
    else if(test == "KVMTest")
        return true;
    else if(test == "AssemblyPegInTube")
        return true;
    else if(test == "MotorTest")
        return true;
    else if(test == "TwoFingerGraspTest")
        return true;
    EngineTest::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
        if(ext.getProperties().get("testID",ext.name) == test)
            return true;
    }
    return false;
}

EngineTest::Ptr EngineTest::Factory::getTest(const std::string& test) {
    if( test == "FreeMotion")
        return new FreeMotionTest();
    else if( test == "CollisionBouncingBallTest")
        return new CollisionBouncingBallTest();
    else if( test == "CollisionBouncingBoxTest")
        return new CollisionBouncingBoxTest();
    else if( test == "CollisionBouncingCylinderTest")
        return new CollisionBouncingCylinderTest();
    else if( test == "ConstraintRevoluteTest")
        return new ConstraintRevoluteTest();
    else if( test == "BowlFeederDrop")
        return new BowlFeederDropTest();
    else if( test == "BallDropTest")
        return new BallDropTest();
    else if( test == "StackingBallsTest")
        return new StackingBallsTest();
    else if( test == "IntegratorRotationTest")
        return new IntegratorRotationTest();
    else if( test == "IntegratorSpringTest")
        return new IntegratorSpringTest();
    else if( test == "FrictionSlidingTest")
    	return new FrictionSlidingTest();
    else if( test == "FrictionStaticTest")
    	return new FrictionStaticTest();
    else if( test == "PiHCompliantTest")
        return new PiHCompliantTest();
    else if( test == "TubePlaneTest")
        return new TubePlaneTest();
    else if( test == "KVMDropTest")
        return new KVMDropTest();
    else if( test == "KVMNutPlaneTest")
        return new KVMNutPlaneTest();
    else if( test == "KVMTest")
        return new KVMTest();
    else if( test == "AssemblyPegInTube")
        return new AssemblyPegInTube();
    else if( test == "MotorTest")
        return new MotorTest();
    else if( test == "TwoFingerGraspTest")
        return new TwoFingerGraspTest();
    EngineTest::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	BOOST_FOREACH(Extension::Ptr& ext, exts){
		const PropertyMap& props = ext->getProperties();
		if(props.get("testID",ext->getName() ) == test){
			return ext->getObject().cast<EngineTest>();
		}
	}
	return NULL;
}
