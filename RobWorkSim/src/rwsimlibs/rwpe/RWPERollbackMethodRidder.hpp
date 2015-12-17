/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_RWPE_RWPEROLLBACKMETHODRIDDER_HPP_
#define RWSIMLIBS_RWPE_RWPEROLLBACKMETHODRIDDER_HPP_

/**
 * @file RWPERollbackMethodRidder.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPERollbackMethodRidder
 */

#include "RWPERollbackMethod.hpp"

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief Ridders Method for doing rollback.
 */
class RWPERollbackMethodRidder: public rwsimlibs::rwpe::RWPERollbackMethod {
public:
	//! @brief Constructor.
	RWPERollbackMethodRidder();

	//! @brief Destructor.
	virtual ~RWPERollbackMethodRidder();

	//! @copydoc RWPERollbackMethod::createData
	RollbackData* createData() const;

	//! @copydoc RWPERollbackMethod::getTimestep
	double getTimestep(SampleSet& samples, RollbackData* data) const;

private:
	class RollbackDataRidder;
	static double threeSamples(const Sample& sample1, const Sample& sample2, const Sample& sample3, RollbackDataRidder* data);
	static void fixSampleSet(SampleSet& samples);
	static std::pair<const rw::kinematics::Frame*, const rw::kinematics::Frame*> findDeepest(const Sample& sample);
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEROLLBACKMETHODRIDDER_HPP_ */