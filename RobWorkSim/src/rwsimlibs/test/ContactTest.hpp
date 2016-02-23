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

#ifndef RWSIMLIBS_TEST_CONTACTTEST_HPP_
#define RWSIMLIBS_TEST_CONTACTTEST_HPP_

/**
 * @file ContactTest.hpp
 *
 * \copydoc rwsimlibs::test::ContactTest
 */

#include <rw/common/ExtensionPoint.hpp>

namespace rw { namespace models { class WorkCell; } }

namespace rwsimlibs {
namespace test {
//! @addtogroup INSERT_DOC_GROUP

//! @{
/**
 * @brief INSERT_SHORT_DESCRIPTION
 */
class ContactTest {
public:
	//! @brief Smart pointer to ContactTest.
	typedef rw::common::Ptr<ContactTest> Ptr;

	//! @brief Construct new test.
	ContactTest();

	//! @brief Destructor.
	virtual ~ContactTest();

	/**
	 * @brief Get the workcell used by the test.
	 * @param map [in] properties for test workcell.
	 * @return a smart pointer to a workcell.
	 */
	virtual rw::common::Ptr<rw::models::WorkCell> getWC(const rw::common::PropertyMap& map) = 0;

	virtual rw::common::Ptr<rw::common::PropertyMap> getDefaultParameters() const;

	/**
	 * @brief A factory for contact detector tests. This factory also defines an ExtensionPoint.
	 */
	class Factory: public rw::common::ExtensionPoint<ContactTest> {
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
		 * @return a pointer to a new ContactTest.
		 */
		static ContactTest::Ptr getTest(const std::string& test);

	private:
		Factory();
	};
};
//! @}
} /* namespace test */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TEST_CONTACTTEST_HPP_ */
