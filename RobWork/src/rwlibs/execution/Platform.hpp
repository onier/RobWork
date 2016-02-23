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

#ifndef RWLIBS_EXECUTION_PLATFORM_HPP_
#define RWLIBS_EXECUTION_PLATFORM_HPP_

/**
 * @file Platform.hpp
 *
 * \copydoc rwlibs::execution::Platform
 */

// Forward declarations
namespace rw { namespace common { template <class T> class Ptr; } }
namespace rw { namespace common { class ThreadTask; } }
namespace rw { namespace models { class Object; } }
namespace rw { namespace sensor { class Sensor; } }
namespace rwlibs { namespace control { class Controller; } }

namespace rwlibs {
namespace execution {
//! @addtogroup execution

//! @{
/**
 * @brief The interface of a platform where tasks can be executed.
 */
class Platform {
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<Platform> Ptr;

	Platform();
	virtual ~Platform() {};

	virtual bool isVirtual() const = 0;

	virtual bool start(rw::common::Ptr<rw::common::ThreadTask> thread) = 0;
	virtual bool pause() = 0;
	virtual bool resume() = 0;
	virtual bool stop() = 0;

	virtual bool isRunning() = 0;
	virtual bool isPaused() = 0;
	virtual bool isFailed() = 0;

	/*std::vector<rw::common::Ptr<rw::sensor::Sensor> > getSensors();
	std::vector<rw::common::Ptr<rwlibs::control::Controller> > getControllers();
	std::vector<rw::common::Ptr<rw::models::Object> > getObjects();*/
};
//! @}
} /* namespace execution */
} /* namespace rwlibs */
#endif /* RWLIBS_EXECUTION_PLATFORM_HPP_ */
