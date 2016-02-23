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

#ifndef RWSIMLIBS_TEST_ENGINETEST_HPP_
#define RWSIMLIBS_TEST_ENGINETEST_HPP_

/**
 * @file EngineTest.hpp
 *
 * \copydoc rwsimlibs::test::EngineTest
 */

#include <rw/common/ExtensionPoint.hpp>
#include <rw/trajectory/Path.hpp>

namespace rw { namespace common { class ThreadTask; } }
namespace rw { namespace common { template <typename T> class ThreadSafeVariable; } }
namespace rwsim { namespace dynamics { class DynamicWorkCell; } }
namespace rwsim { namespace log { class SimulatorLogScope; } }

#include <boost/bind.hpp>

namespace rwsimlibs {
namespace test {
//! @addtogroup INSERT_DOC_GROUP

//! @{
/**
 * @brief INSERT_SHORT_DESCRIPTION
 */
class EngineTest {
public:
	//! @brief Smart pointer to EngineTest.
	typedef rw::common::Ptr<EngineTest> Ptr;

	//! @brief The callback type for the current simulation time
	typedef boost::function<void(double,bool,bool)> TimeCallback;

	struct Result {
		Result(const std::string& name, const std::string& description);
		std::string name;
		std::string description;
		rw::trajectory::TimedQPath values;
	};

	//! @brief Handle for a concrete test run.
	class TestHandle {
	public:
		typedef rw::common::Ptr<TestHandle> Ptr;

		TestHandle();

		virtual ~TestHandle();

		virtual std::string getError() const;

		virtual rw::trajectory::TimedStatePath getTimedStatePath() const;

		virtual const std::vector<Result>& getResults() const;

		virtual void setError(const std::string& error);

		virtual void append(const rw::trajectory::TimedState& tstate);

		virtual void append(const Result& result);

		virtual bool isAborted();

		virtual void abort();

		/**
		 * @brief Set a callback with the current simulation time.
		 * @param cb [in] the function to call.
		 */
		void setTimeCallback(TimeCallback cb);

		void callback(double,bool,bool);

	private:
		std::string _error;
		rw::trajectory::TimedStatePath _path;
		std::vector<Result> _results;
		rw::common::ThreadSafeVariable<bool>* _abort;
		TimeCallback _cb;
	};

	//! @brief Construct new test.
	EngineTest();

	//! @brief Destructor.
	virtual ~EngineTest();

	/**
	 * @brief Check if engine with specific name is supported by the test.
	 * @param engineID [in] the id of the engine.
	 * @return true if supported, false otherwise.
	 */
	virtual bool isEngineSupported(const std::string& engineID) const = 0;

	virtual TestHandle::Ptr runThread(const std::string& engineID, const rw::common::PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose, rw::common::Ptr<rw::common::ThreadTask> task);

	virtual void run(TestHandle::Ptr handle, const std::string& engineID, const rw::common::PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose = NULL) = 0;

	virtual double getRunTime() const = 0;

	/**
	 * @brief Get the dynamic workcell used by the test.
	 * @param map [in] properties for test workcell.
	 * @return a smart pointer to a dynamic workcell.
	 */
	virtual rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> getDWC(const rw::common::PropertyMap& map) = 0;

	virtual rw::common::Ptr<rw::common::PropertyMap> getDefaultParameters() const;

	/**
	 * @brief A factory for engine tests. This factory also defines an ExtensionPoint.
	 */
	class Factory: public rw::common::ExtensionPoint<EngineTest> {
	public:
		/**
		 * @brief Get the available tests.
		 * @return a vector of identifiers for tests.
		 */
		static std::vector<std::string> getTests();

		/**
		 * @brief Check if test is available.
		 * @param test [in] the name of the test.
		 * @return true if available, false otherwise.
		 */
		static bool hasTest(const std::string& test);

		/**
		 * @brief Create a new test.
		 * @param test [in] the name of the test.
		 * @return a pointer to a new EngineTest.
		 */
		static EngineTest::Ptr getTest(const std::string& test);

	private:
		Factory();
	};
};
//! @}
} /* namespace test */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TEST_ENGINETEST_HPP_ */
