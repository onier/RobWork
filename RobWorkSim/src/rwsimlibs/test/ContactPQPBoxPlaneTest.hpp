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

#ifndef RWSIMLIBS_TEST_CONTACTPQPBOXPLANETEST_HPP_
#define RWSIMLIBS_TEST_CONTACTPQPBOXPLANETEST_HPP_

/**
 * @file ContactPQPBoxPlaneTest.hpp
 *
 * \copydoc rwsimlibs::test::ContactPQPBoxPlaneTest
 */

#include "ContactTest.hpp"

namespace rwsimlibs {
namespace test {
//! @addtogroup INSERT_DOC_GROUP

//! @{
/**
 * @brief INSERT_SHORT_DESCRIPTION
 */
class ContactPQPBoxPlaneTest: public ContactTest {
public:
	//! @brief Constructor.
	ContactPQPBoxPlaneTest();

	//! @brief Destructor.
	virtual ~ContactPQPBoxPlaneTest();

	//! @copydoc ContactTest::getWC
	virtual rw::common::Ptr<rw::models::WorkCell> getWC(const rw::common::PropertyMap& map);

	//! @copydoc ContactTest::getPoses
	virtual std::map<std::string, rw::kinematics::State> getPoses(const rw::common::PropertyMap& map);

	//! @copydoc ContactTest::getDefaultParameters
	virtual rw::common::Ptr<rw::common::PropertyMap> getDefaultParameters() const;
};
//! @}
} /* namespace test */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TEST_CONTACTPQPBOXPLANETEST_HPP_ */
