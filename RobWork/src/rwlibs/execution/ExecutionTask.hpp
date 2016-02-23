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

#ifndef RWLIBS_EXECUTION_EXECUTIONTASK_HPP_
#define RWLIBS_EXECUTION_EXECUTIONTASK_HPP_

/**
 * @file ExecutionTask.hpp
 *
 * \copydoc rwlibs::execution::ExecutionTask
 */

#include <rw/common/Ptr.hpp>
#include <rw/common/Serializable.hpp>

namespace rwlibs {
namespace execution {
//! @addtogroup execution

//! @{
/**
 * @brief A generic interface for a task that can be executed in a Executor and saved in a TaskDB.
 */
class ExecutionTask: public rw::common::Serializable {
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<ExecutionTask> Ptr;

	ExecutionTask();
	virtual ~ExecutionTask();

	/**
	 * @brief Make a copy of the task.
	 * @return a pointer to a new cloned ExecutionTask.
	 */
	virtual ExecutionTask::Ptr clone() const = 0;

	/**
	 * @name Serialization
	 * @brief A ExecutionTask should always be serializable.
	 */
	///@{
	//! @copydoc rw::common::Serializable::read
	virtual void read(class rw::common::InputArchive& iarchive, const std::string& id) = 0;
	//! @copydoc rw::common::Serializable::write
	virtual void write(class rw::common::OutputArchive& iarchive, const std::string& id) const = 0;
	///@}

public:
	//! A unique id identifying the task
	std::string taskID;
};
//! @}
} /* namespace execution */
} /* namespace rwlibs */
#endif /* RWLIBS_EXECUTION_EXECUTIONTASK_HPP_ */
