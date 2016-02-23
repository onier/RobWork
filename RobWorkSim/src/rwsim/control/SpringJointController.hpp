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

#ifndef RWSIM_CONTROL_SPRINGJOINTCONTROLLER_HPP_
#define RWSIM_CONTROL_SPRINGJOINTCONTROLLER_HPP_

/**
 * @file SpringJointController.hpp
 *
 * \copydoc rwsim::control::SpringJointController
 */

#include <rwlibs/control/JointController.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>

namespace rwsim {

namespace dynamics { class RigidDevice; }

namespace control {
	//! @addtogroup rwsim_control
	//! @{

	/**
	 * @brief The spring joint controller controls a rigid device by applying a force in each of the joints.
	 */
	class SpringJointController: public rwlibs::control::JointController, public rwlibs::simulation::SimulatedController {
	public:
	    //! @brief Parameters of a 1 dof spring.
	    struct SpringParam {
	    	//! @brief Constructor.
	    	SpringParam(): elasticity(0), dampening(0), offset(0) {}
	    	//! @brief The coefficient that gives the force as a proportion of the relative displacement of the joint.
	        double elasticity;
	        //! @brief The coefficient that adds dampening based on the relative velocity in the joint.
	        double dampening;
	        //! @brief An offset to to the relative displacement.
	        double offset;
	    };

	    //! @brief Smart pointer type.
	    typedef rw::common::Ptr<SpringJointController> Ptr;

		/**
		 * @brief Constructor.
		 * @param name [in] the name of the controller.
		 * @param rdev [in] the rigid device to be controlled.
		 * @param springParam [in] list of spring parameters. List must be the same length as the number of joints.
		 */
		SpringJointController(
		        const std::string& name,
		        rw::common::Ptr<rwsim::dynamics::RigidDevice> rdev,
				const std::vector<SpringParam>& springParam);

		//! @brief Destructor.
		virtual ~SpringJointController(){};

		//! @copydoc SimulatedController::update
		void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);

		//! @copydoc SimulatedController::reset
		void reset(const rw::kinematics::State& state);

		//! @copydoc SimulatedController::getControllerName
		std::string getControllerName() { return getName(); }

		//! @copydoc SimulatedController::setEnabled
        void setEnabled(bool enabled) { _enabled = enabled; }

		//! @copydoc SimulatedController::isEnabled
        bool isEnabled() const { return _enabled; }

		////// inherited from JointController

		/**
		 * @copydoc JointController::getControlModes
		 *
		 * This controller supports only position control.
		 */
		unsigned int getControlModes();

		//! @copydoc JointController::setControlModes
		void setControlMode(ControlMode mode);

		//! @copydoc JointController::setTargetPos
		void setTargetPos(const rw::math::Q& target);

		//! @copydoc JointController::setTargetVel
		void setTargetVel(const rw::math::Q& vals);

		//! @copydoc JointController::setTargetAcc
		void setTargetAcc(const rw::math::Q& vals);

		//! @copydoc JointController::getQ
		rw::math::Q getQ(){ return _currentQ;}

		//! @copydoc JointController::getQd
		rw::math::Q getQd(){ return rw::math::Q();}

		//! @copydoc SimulatedController::getControllerHandle
        rwlibs::control::Controller::Ptr getControllerHandle(rwlibs::simulation::Simulator::Ptr sim){ return this; }

	private:
		SpringJointController();

	private:
		rw::common::Ptr<rwsim::dynamics::RigidDevice> _ddev;
		rw::math::Q _target, _currentQ;
		rw::math::Q _qError;
		std::vector<SpringParam> _springParams;
		bool _enabled;
	};

	//! @}
}
}

#endif /*RWSIM_CONTROL_SPRINGJOINTCONTROLLER_HPP_*/
