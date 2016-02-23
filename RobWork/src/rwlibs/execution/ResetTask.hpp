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

#ifndef RESETTASK_HPP_
#define RESETTASK_HPP_

/**
 * @file ResetTask.hpp
 *
 * \copydoc rwlibs::execution::ResetTask
 */

#include "ExecutionTask.hpp"

namespace rwlibs {
namespace execution {
//! @addtogroup execution

//! @{
/**
 * @brief A task that defines an operation that resets a environment.
 */
class ResetTask: public rwlibs::execution::ExecutionTask {
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<ResetTask> Ptr;

    //! @brief Constructor
	ResetTask();
	//! @brief Destructor
	virtual ~ResetTask();

    //! @copydoc rwlibs::execution::ExecutionTask::clone
	virtual ExecutionTask::Ptr clone() const;

	/**
	 * @name Serialization
	 * @brief A ExecutionTask should always be serializable.
	 */
	///@{
    //! @copydoc rwlibs::execution::ExecutionTask::read
	virtual void read(class rw::common::InputArchive& iarchive, const std::string& id);
    //! @copydoc rwlibs::execution::ExecutionTask::write
	virtual void write(class rw::common::OutputArchive& iarchive, const std::string& id) const;
	///@}
};
//! @}
} /* namespace execution */
} /* namespace rwlibs */
#endif /* RESETTASK_HPP_ */
