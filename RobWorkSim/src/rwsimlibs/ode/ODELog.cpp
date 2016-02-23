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

#include "ODELog.hpp"
#include "ODEBody.hpp"

#include <rwsim/contacts/Contact.hpp>
#include <rwsim/dynamics/Body.hpp>
#include <rwsim/log/SimulatorLogScope.hpp>
#include "../../rwsim/log/LogContactSet.hpp"
#include "../../rwsim/log/LogMessage.hpp"
#include "../../rwsim/log/LogPositions.hpp"
#include "../../rwsim/log/LogStep.hpp"

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::log;
using namespace rwsim::simulator;

ODELog::ODELog(SimulatorLogScope::Ptr log):
	_log(log),
	_scope(log.get())
{
	RW_ASSERT(!log.isNull());
}

ODELog::~ODELog() {
}

void ODELog::beginStep(double time, const char* file, int line) {
	LogStep::Ptr step = ownedPtr(new LogStep(_scope));
	_scope->appendChild(step);
	step->setTimeBegin(time);
	step->setFilename(file);
	step->setLineBegin(line);
	_scope = step.get();
}

void ODELog::endStep(double time, int line) {
	LogStep* const step = dynamic_cast<LogStep*>(_scope);
	if (step == NULL)
		RW_THROW("Could not end step! - Not in correct scope.");
	step->setTimeEnd(time);
	step->setLineEnd(line);
	_scope = step->getParent();
}

void ODELog::addContacts(const std::string& description, const std::vector<rwsim::contacts::Contact>& contacts, const char* file, int line) {
	LogContactSet::Ptr entry = ownedPtr(new LogContactSet(_scope));
	_scope->appendChild(entry);
	entry->setDescription(description);
	entry->setContacts(contacts);
	entry->setFilename(file);
	entry->setLine(line);
	entry->autoLink();
}

void ODELog::addPositions(const std::string& description, const std::vector<ODEBody*>& bodies, const State& state, const char* file, int line) {
	LogPositions::Ptr entry = ownedPtr(new LogPositions(_scope));
	_scope->appendChild(entry);
	entry->setDescription(description);
	std::map<std::string, Transform3D<> > positions;
	BOOST_FOREACH(ODEBody* const body, bodies) {
		positions[body->getRwBody()->getName()] = body->getRwBody()->wTbf(state);
	}
	entry->setPositions(positions);
	entry->setFilename(file);
	entry->setLine(line);
	entry->autoLink();
}

std::ostream& ODELog::log(const std::string& description, const char* file, int line) {
	LogMessage::Ptr entry = ownedPtr(new LogMessage(_scope));
	_scope->appendChild(entry);
	entry->setDescription(description);
	entry->setFilename(file);
	entry->setLine(line);
	entry->autoLink();
	return entry->stream();
}
