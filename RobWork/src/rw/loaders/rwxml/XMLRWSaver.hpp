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

#ifndef RW_LOADERS_XMLRWSAVER_HPP_
#define RW_LOADERS_XMLRWSAVER_HPP_

/**
 * @file XMLRWSaver.hpp
 *
 * \copydoc rw::loaders::XMLRWSaver
 */

#include <rw/models/WorkCell.hpp>

namespace rw {
namespace loaders {
//! @addtogroup loaders

//! @{
/**
 * @brief Save a WorkCell to a RW XML file.
 *
 * @note This is still under development - be careful.
 */
class XMLRWSaver {
public:
	/**
	 * @brief Store a WorkCell into a RW XML file.
	 * @param workcell [in] the WorkCell to store.
	 * @param filename [in] the filename to save to.
	 */
	static void save(rw::models::WorkCell::Ptr workcell, const std::string& filename);

private:
	XMLRWSaver() {};
	virtual ~XMLRWSaver() {};
};
//! @}
} /* namespace loaders */
} /* namespace rw */
#endif /* RW_LOADERS_XMLRWSAVER_HPP_ */
