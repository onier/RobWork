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

#ifndef RWSIMLIBS_TEST_INTEGRATORROTATIONTEST_HPP_
#define RWSIMLIBS_TEST_INTEGRATORROTATIONTEST_HPP_

/**
 * @file IntegratorRotationTest.hpp
 *
 * \copydoc rwsimlibs::test::IntegratorRotationTest
 */

#include "EngineTest.hpp"

namespace rwsimlibs {
namespace test {
//! @addtogroup INSERT_DOC_GROUP

//! @{
/**
 * @brief INSERT_SHORT_DESCRIPTION
 */
class IntegratorRotationTest: public EngineTest {
public:
	//! @brief Smart pointer to IntegratorRotationTest.
	typedef rw::common::Ptr<IntegratorRotationTest> Ptr;

	/**
	 * @brief Constructor.
	 * @param integratorType [in] the type of integrator to use (optional).
	 */
	IntegratorRotationTest(const std::string& integratorType = "");

	//! @brief Destructor.
	virtual ~IntegratorRotationTest();

	//! @copydoc EngineTest::isEngineSupported
	virtual bool isEngineSupported(const std::string& engineID) const;

	//! @copydoc EngineTest::run
	virtual void run(TestHandle::Ptr handle, const std::string& engineID, const rw::common::PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose = NULL);

	//! @copydoc EngineTest::getRunTime
	virtual double getRunTime() const;

	//! @copydoc EngineTest::getDWC
	virtual rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> getDWC(const rw::common::PropertyMap& map);

	//! @copydoc EngineTest::getDefaultParameters
	virtual rw::common::Ptr<rw::common::PropertyMap> getDefaultParameters() const;

	/**
	 * @brief Get the expected kinetic energy.
	 * @param dwc [in] the dynamic workcell.
	 * @return the energy.
	 */
	double getExpectedEnergy(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc) const;

private:
	const std::string _integratorType;
	rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> _dwc;
};
//! @}
} /* namespace test */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TEST_INTEGRATORROTATIONTEST_HPP_ */
