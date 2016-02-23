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

#ifndef ROBWORKSIM_SRC_RWSIMLIBS_TEST_INTEGRATORSPRINGTEST_HPP_
#define ROBWORKSIM_SRC_RWSIMLIBS_TEST_INTEGRATORSPRINGTEST_HPP_

/**
 * @file IntegratorSpringTest.hpp
 *
 * \copydoc rwsimlibs::test::IntegratorSpringTest
 */

#include "EngineTest.hpp"

namespace rwsimlibs {
namespace test {
//! @addtogroup INSERT_DOC_GROUP

//! @{
/**
 * @brief INSERT_SHORT_DESCRIPTION
 */
class IntegratorSpringTest: public EngineTest {
public:
	/**
	 * @brief Constructor.
	 * @param integratorType [in] the type of integrator to use (optional).
	 */
	IntegratorSpringTest(const std::string& integratorType = "");

	//! @brief Destructor.
	virtual ~IntegratorSpringTest();

	//! @copydoc EngineTest::isEngineSupported
	virtual bool isEngineSupported(const std::string& engineID) const;

	//! @copydoc EngineTest::run
	virtual void run(TestHandle::Ptr handle, const std::string& engineID, const rw::common::PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose = NULL);

	//! @copydoc EngineTest::getRunTime
	virtual double getRunTime() const;

	//! @copydoc EngineTest::getDWC
	virtual rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> getDWC(const rw::common::PropertyMap& map);

	/**
	 * @brief Create new dynamic workcell.
	 * @param integratorType [in] (optional) the integrator to use.
	 * @return the dynamic workcell.
	 */
	static rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> makeDWC(const std::string& integratorType = "");

	/**
	 * @brief Get reference position.
	 * @param t [in] the time.
	 * @return the reference position.
	 */
	double reference(double t) const;

private:
	const std::string _integratorType;
	rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> _dwc;
};
//! @}
} /* namespace test */
} /* namespace rwsimlibs */
#endif /* ROBWORKSIM_SRC_RWSIMLIBS_TEST_INTEGRATORSPRINGTEST_HPP_ */
