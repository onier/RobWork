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

#ifndef RWSIMLIBS_ODE_ODELOG_HPP_
#define RWSIMLIBS_ODE_ODELOG_HPP_

//! @brief Macro that works as two arguments for file and line.
#define LOG_LOCATION __FILE__,__LINE__

/**
 * @file ODELog.hpp
 *
 * \copydoc rwsim::simulator::ODELog
 */

#include <rw/common/Ptr.hpp>

namespace rw { namespace kinematics { class State; } }
namespace rwsim { namespace contacts { class Contact; } }
namespace rwsim { namespace log { class SimulatorLogScope; } }

namespace rwsim {
namespace simulator {

class ODEBody;

//! @addtogroup rwsim_simulator

//! @{
/**
 * @brief Logging utility for ODE.
 */
class ODELog {
public:
	ODELog(rw::common::Ptr<rwsim::log::SimulatorLogScope> log);
	virtual ~ODELog();

	virtual void beginStep(double time, const char* file = "", int line = -1);

	virtual void endStep(double time, int line = -1);

	virtual void addContacts(const std::string& description, const std::vector<rwsim::contacts::Contact>& contacts, const char* file = "", int line = -1);
	virtual void addPositions(const std::string& description, const std::vector<ODEBody*>& bodies, const rw::kinematics::State& state, const char* file = "", int line = -1);

	virtual std::ostream& log(const std::string& description, const char* file = "", int line = -1);

private:
	rw::common::Ptr<rwsim::log::SimulatorLogScope> _log;
	rwsim::log::SimulatorLogScope* _scope;
};
//! @}
} /* namespace simulator */
} /* namespace rwsim */
#endif /* RWSIMLIBS_ODE_ODELOG_HPP_ */
