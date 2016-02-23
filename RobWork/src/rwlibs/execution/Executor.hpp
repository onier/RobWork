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

#ifndef RWLIBS_EXECUTION_EXECUTOR_HPP_
#define RWLIBS_EXECUTION_EXECUTOR_HPP_

/**
 * @file Executor.hpp
 *
 * \copydoc rwlibs::execution::Executor
 */

#include <rw/common/Ptr.hpp>

// Forward declarations
namespace rw { namespace common { class ThreadTask; }}

namespace rwlibs {
namespace execution {

// Forward declarations
class ExecutionTask;
class Platform;

//! @addtogroup execution

//! @{
/**
 * @brief A generic interface for implementation of execution loops that can execute tasks.
 *
 * An execution loop must be associated to one platform where it can execute the task.
 *
 * When a task is executed it is possible to do this in a ThreadPool, by giving it a
 * ThreadTask to add its work to.
 */
class Executor {
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<Executor> Ptr;

    /**
     * @brief Start a task execution loop that runs on the given platform.
     * @param platform [in] the platform to execute the task on.
     */
	Executor(rw::common::Ptr<Platform> platform);

	//! @brief Destructor
	virtual ~Executor();

	/**
	 * @brief Start execution of a new ExecutionTask.
	 * @param task [in] the execution task to run.
	 * @param thread [in] (optional) the ThreadTask to add work to (if Executor should run in a separate thread)
	 * @return true if task started successfully, false if another task was already running on the platform or task type was not recognized.
	 */
	virtual bool start(rw::common::Ptr<ExecutionTask> task, rw::common::Ptr<rw::common::ThreadTask> thread) = 0;

	/**
	 * @brief Check if a task is already executing.
	 * @return true if Executor is busy, false otherwise.
	 */
	virtual bool isRunning() = 0;

protected:
	rw::common::Ptr<Platform> _platform;
};
//! @}
} /* namespace execution */
} /* namespace rwlibs */
#endif /* RWLIBS_EXECUTION_EXECUTOR_HPP_ */
