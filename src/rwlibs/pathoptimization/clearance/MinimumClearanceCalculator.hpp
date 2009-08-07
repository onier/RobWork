/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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


#ifndef RWLIBS_PATHOPTIMIZATION_MINIMUMCLEARANCECALCULATOR_HPP
#define RWLIBS_PATHOPTIMIZATION_MINIMUMCLEARANCECALCULATOR_HPP

#include "ClearanceCalculator.hpp"

#include <boost/shared_ptr.hpp>
#include <rw/proximity/DistanceCalculator.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/WorkCell.hpp>

namespace rwlibs {
namespace pathoptimization {

/** @addtogroup pathoptimization */
/*@{*/

/**
 * @brief Implements a MinimumClearanceCalculator
 *
 * The minimum clearance is defined as the minimal distance between any two geometries, which are not excluded
 * by the collision setup.
 */
class MinimumClearanceCalculator: public ClearanceCalculator
{
public:
    /**
     * @brief Constructs a MinimumClearanceCalculator using the \b DistanceCalculator provided.
     *
     * Use this constructor to use an already existing DistanceCalculator
     *
     * @param distancecalculator [in] The distance calculator to use
     */
	MinimumClearanceCalculator(rw::proximity::DistanceCalculatorPtr distancecalculator);

	/**
	 * @brief Constructs a MinimumClearanceCalculator for a workcell
	 * @param workcell [in] WorkCell for which to calculate the minimum clearance
	 * @param state [in] State of the workcell
	 */
	MinimumClearanceCalculator(rw::models::WorkCellPtr workcell,
	                           const rw::kinematics::State& state);

	/**
	 * @brief Destructor
	 */
	virtual ~MinimumClearanceCalculator();

	/**
	 * @copydoc ClearanceCalculator::clearance
	 */
	double clearance(rw::kinematics::State& state);

private:
    rw::proximity::DistanceCalculatorPtr _distancecalculator;
};


 /** @} */
} //end pathoptimization
} //end rwlibs

#endif //#ifndef RWLIBS_PATHOPTIMIZATION_MINIMUMCLEARANCECALCULATOR_HPP
