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

#ifndef RWLIBS_EXECUTION_TASKDB_HPP_
#define RWLIBS_EXECUTION_TASKDB_HPP_

/**
 * @file TaskDB.hpp
 *
 * \copydoc rwlibs::execution::TaskDB
 */

#include <rw/common/Ptr.hpp>

#include <vector>

namespace rwlibs {
namespace execution {

// Forward declarations
class ExecutionResult;
class ExecutionTask;

//! @addtogroup execution

//! @{
/**
 * @brief Interface for databases that store ExecutionTask and associated ExecutionResult data.
 *
 * A database must implement different methods for adding, removing and retrieving tasks and results.
 * Furthermore the database should provide some basic statistics.
 *
 * Implementations of this interface must be thread safe.
 */
class TaskDB {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<TaskDB> Ptr;

	//! @brief Construct new empty database.
	TaskDB() {};

	//! @brief Destruct database.
	virtual ~TaskDB() {};

	/**
	 * @brief Insert data from other database into this database.
	 * @param db [in] pointer to database with original data.
	 */
	virtual void addDB(TaskDB::Ptr db) = 0;

	/**
	 * @name Tasks
	 * @brief Functions for accessing and changing the tasks in the database.
	 */
	///@{
	/**
	 * @brief Add a new task to the database.
	 * @param task [in] the task to add.
	 */
	virtual void addTask(rw::common::Ptr<ExecutionTask> task) = 0;

	/**
	 * @brief Add a list of tasks to the database.
	 * @param tasks [in] the list of tasks to add.
	 */
	virtual void addTasks(const std::vector<rw::common::Ptr<ExecutionTask> > &tasks) = 0;

	/**
	 * @brief Get a specific task from the database.
	 * @param i [in] the id of the task to retrieve.
	 */
	virtual std::vector<rw::common::Ptr<ExecutionTask> > getTask(unsigned int i) const = 0;

	/**
	 * @brief Get all tasks in the database as a list.
	 * @param task [in] the task to add.
	 */
	virtual std::vector<rw::common::Ptr<ExecutionTask> > getTasks() const = 0;
	virtual rw::common::Ptr<ExecutionTask> getRandomTask() const = 0;
	virtual rw::common::Ptr<ExecutionTask> getNewTaskCycled() = 0;
    ///@}

	/**
	 * @name Results
	 * @brief Functions for accessing and changing the results in the database.
	 */
	///@{
	virtual void addResult(const rw::common::Ptr<ExecutionResult> result) = 0;
	virtual void addResults(const std::vector<rw::common::Ptr<ExecutionResult> > &results) = 0;
	virtual std::vector<rw::common::Ptr<ExecutionResult> > getResults() = 0;
    ///@}

	/**
	 * @name Statistics
	 * @brief Functions for retrieving statistics from the database.
	 */
	///@{
	virtual std::vector<unsigned int> getStatisticsSum() const = 0;
	virtual std::vector<unsigned int> getStatisticsNonAssigned() const = 0;
	virtual std::vector<unsigned int> getStatisticsAssigned(std::size_t taskId) const = 0;
	virtual std::vector<std::vector<unsigned int> > getStatisticsAssigned() const = 0;
    ///@}
};
//! @}
} /* namespace execution */
} /* namespace rwlibs */
#endif /* RWLIBS_EXECUTION_TASKDB_HPP_ */
