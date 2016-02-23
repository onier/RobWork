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

#ifndef RWSIM_CONTACTS_BALLMODEL_HPP_
#define RWSIM_CONTACTS_BALLMODEL_HPP_

/**
 * @file BallModel.hpp
 *
 * \copydoc rwsim::contacts::BallModel
 */

#include "ContactModel.hpp"

namespace rwsim {
namespace contacts {
//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief Contact model of a ball - used for the contact strategies.
 */
class BallModel: public ContactModel {
public:
	//! @brief Smart pointer type
	typedef rw::common::Ptr<BallModel> Ptr;

	/**
	 * @brief Constructor.
	 * @param owner [in] the strategy that owns this model.
	 */
	BallModel(ContactStrategy* owner): ContactModel(owner) {}

	//! @copydoc ContactModel::getName
	virtual std::string getName() const { return "BallModel"; }

	std::vector<Model> models;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_BALLMODEL_HPP_ */
