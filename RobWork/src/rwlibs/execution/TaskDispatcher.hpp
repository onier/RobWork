/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef TASKDISPATCHER_HPP_
#define TASKDISPATCHER_HPP_

/**
 * @file TaskDispatcher.hpp
 *
 * \copydoc rwlibs::execution::TaskDispatcher
 */

namespace rwlibs {
namespace execution {
//! @addtogroup INSERT_DOC_GROUP

//! @{
/**
 * @brief INSERT_SHORT_DESCRIPTION
 */
class TaskDispatcher {
public:
	TaskDispatcher();
	virtual ~TaskDispatcher();

	bool setTaskDB(rw::common::Ptr<TaskDB> db);

	virtual void start(rw::common::Ptr<rw::common::ThreadTask> task) = 0;
	virtual bool isRunning() = 0;

private:
	// db
	// platforms
};
//! @}
} /* namespace execution */
} /* namespace rwlibs */
#endif /* TASKDISPATCHER_HPP_ */
