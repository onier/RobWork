#ifndef RWSIM_CONTROL_SpringJointController_HPP_
#define RWSIM_CONTROL_SpringJointController_HPP_

//! @file BeamJointController.hpp

#include <rwlibs/control/JointController.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>

namespace rwsim { namespace dynamics { class RigidDevice; } }

namespace rwsim {
namespace control {
	//! @addtogroup rwsim_control
	//! @{

	/**
	 * @brief
	 *
	 */
	class SpringJointController: public rwlibs::control::JointController, public rwlibs::simulation::SimulatedController {
	public:
	    //! @brief parameters of a 1 dof spring
	    struct SpringParam {
	        double elasticity; //
	        double dampening;
	        double offset;
	    };

	    typedef rw::common::Ptr<SpringJointController> Ptr;

		/**
		 * @brief constructor
		 * @param name
		 * @param rdev [in] device that is to be controlled
		 * @param springParam
		 * @cond
		 * @param state [in] target state
		 * @param cmode [in] the control mode used
		 * @param pdparams [in] list of pd parameters. must be same length as number of joints.
		 * @endcond
		 * @param dt [in] the sampletime (time between samples in seconds) used in the control
		 * loop, this should be larger than the expected update sample time.
		 */
		SpringJointController(
		        const std::string& name,
		        rw::common::Ptr<rwsim::dynamics::RigidDevice> rdev,
				const std::vector<SpringParam>& springParam,
				double dt
				);


		/**
		 * @brief destructor
		 */
		virtual ~SpringJointController(){};


		/**
		 * @brief the time between samples
		 * @return the sample time in seconds
		 */
		double getSampleTime();

		/**
		 * @brief set the time between samples in seconds
		 * @param stime [in] sample time
		 */
		void setSampleTime(double stime);

		//! @copydoc rwlibs::simulation::SimulatedController::update
		void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);

		//! @copydoc rwlibs::simulation::SimulatedController::reset
		void reset(const rw::kinematics::State& state);

		//! @copydoc rwlibs::simulation::SimulatedController::getControllerName
		Controller* getController(){ return this; };

		std::string getControllerName(){ return getName(); };

        void setEnabled(bool enabled){ _enabled = enabled; };

        bool isEnabled() const { return _enabled; } ;

		////// inherited from JointController

		/**
		 * @copydoc rwlibs::control::JointController::getControlModes
		 *
		 * This controller supports both position and velocity control.
		 */
		unsigned int getControlModes(){return _mode;}

		//! @copydoc rwlibs::control::JointController::setControlMode
		void setControlMode(ControlMode mode);

		//! @copydoc rwlibs::control::JointController::setTargetPos
		void setTargetPos(const rw::math::Q& target);

		//! @copydoc rwlibs::control::JointController::setTargetVel
		void setTargetVel(const rw::math::Q& vals);

		//! @copydoc rwlibs::control::JointController::setTargetAcc
		void setTargetAcc(const rw::math::Q& vals);

		//! @copydoc rwlibs::control::JointController::getQ
		rw::math::Q getQ(){ return _currentQ;}

		//! @copydoc rwlibs::control::JointController::getQd
		rw::math::Q getQd(){ return _currentVel;}

        rwlibs::control::Controller::Ptr getControllerHandle(rwlibs::simulation::Simulator::Ptr sim){ return this; }


	private:
		SpringJointController();

	private:
		//std::vector<rw::models::BeamJoint*> _beamJoints;

		rw::common::Ptr<rwsim::dynamics::RigidDevice> _ddev;
		rw::math::Q _maxVel;
		rw::math::Q _lastError, _target, _currentQ, _currentVel;
		rw::math::Q _targetVel;
		ControlMode _mode;
		double _stime, _accTime; // sample time
		rw::math::Q  _P, _D, _qError;
		std::vector<SpringParam> _springParams;
		bool _enabled;
	};

	//! @}
}
}

#endif /*BeamJointController_HPP_*/
