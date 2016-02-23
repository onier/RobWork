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

#ifndef RWSIM_SENSOR_SIMULATEDFTSENSOR_HPP_
#define RWSIM_SENSOR_SIMULATEDFTSENSOR_HPP_

#include "SimulatedTactileSensor.hpp"
#include <rw/sensor/FTSensor.hpp>
#include <rw/sensor/FTSensorModel.hpp>


namespace rwsim {
namespace sensor {
	//! @addtogroup sensor
	//! @{

	/**
	 * @brief A sensor that measures force and torque between two bodies around
	 * some reference frame. Keep in mind that there can be multiple constraints and contacts
	 * between two bodies at the same time. The force measured by this sensor will be
	 * the total force acting between the two bodies. It is not yet possible to measure
	 * the effect of a single constraint. In this case the user must define only one constraint
	 * between the body pair and disable collision detection.
	 */
	class SimulatedFTSensor: public SimulatedTactileSensor {
	public:
		typedef rw::common::Ptr<SimulatedFTSensor> Ptr;
		
		/**
		 * @brief Constructor - the forces acting on \b body1 due to \b body will be stored.
		 * The forces will be described relative to \b body1 if no reference \b \frame is given.
		 *
		 * In case one wants to measure the constraint force between a pair of objects, the
		 * reference frame should be a frame located where the Constraint is acting.
		 *
		 * @param name [in] identifier
		 * @param body [in] the first body
		 * @param body1 [in] the second body
		 * @param frame [in] (optional) the reference frame - default is the \b body1 body frame.
		 */
        SimulatedFTSensor(const std::string& name,
                          dynamics::Body::Ptr body,
                          dynamics::Body::Ptr body1,
                          rw::kinematics::Frame* frame=NULL);


		//! @brief destructor
		virtual ~SimulatedFTSensor();

		//// Interface inherited from SimulatedSensor
		//! @copydoc SimulatedSensor::update
		void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);

		//! @copydoc SimulatedSensor::reset
		void reset(const rw::kinematics::State& state);

		//// Interface inherited from SimulatedTactileSensor
		//! @copydoc SimulatedTactileSensor::addForceW
		void addForceW(const rw::math::Vector3D<>& point,
					   const rw::math::Vector3D<>& force,
					   const rw::math::Vector3D<>& cnormal,
					   rw::kinematics::State& state,
					   dynamics::Body::Ptr body = NULL);

		//! @copydoc SimulatedTactileSensor::addForce
		void addForce(const rw::math::Vector3D<>& point,
					  const rw::math::Vector3D<>& force,
					  const rw::math::Vector3D<>& cnormal,
					  rw::kinematics::State& state,
					  dynamics::Body::Ptr body=NULL);

		//! @copydoc SimulatedTactileSensor::addWrenchToCOM
        void addWrenchToCOM(
                      const rw::math::Vector3D<>& force,
                      const rw::math::Vector3D<>& torque,
                      rw::kinematics::State& state,
                      dynamics::Body::Ptr body=NULL);

		//! @copydoc SimulatedTactileSensor::addWrenchWToCOM
        void addWrenchWToCOM(
                      const rw::math::Vector3D<>& force,
                      const rw::math::Vector3D<>& torque,
                      rw::kinematics::State& state,
                      dynamics::Body::Ptr body=NULL);

        /**
         * @brief Get the transform.
         * @return the transform.
         */
		rw::math::Transform3D<> getTransform() const;

        /**
         * @brief Get the measured force in the reference frame.
		 * @param state [in] the state.
         * @return the force.
         */
		rw::math::Vector3D<> getForce(const rw::kinematics::State& state) const;

		/**
		 * @brief Get the measured torque around and in the reference frame.
		 * @param state [in] the state.
		 * @return the torque.
		 */
		rw::math::Vector3D<> getTorque(const rw::kinematics::State& state) const;

		/**
		 * @brief Maximum torque.
		 * @return the maximum torque.
		 */
		rw::math::Vector3D<> getMaxTorque() const {return _ftmodel->getMaxTorque();}

		/**
		 * @brief Maximum force.
		 * @return the maximum force.
		 */
		rw::math::Vector3D<> getMaxForce() const {return _ftmodel->getMaxForce();}

		/**
		 * @brief Get the sensor frame (the \b body1 body frame unless reference frame is set differently).
		 * @return a pointer to the sensor frame.
		 */
		rw::kinematics::Frame * getSensorFrame() const { return _sframe; }

 		void acquire(){}

		/**
		 * @brief Get a FTSensor version of this sensor.
		 * @param sim [in] the simulator in which the simulated sensor is instantiated
		 * @return a smart pointer to a FTSensor
		 */
		 rw::sensor::FTSensor::Ptr getFTSensor(rwlibs::simulation::Simulator::Ptr sim);

		/**
		 * @brief Get the first body (the body influencing the sensor body)
		 * @return the Body.
		 */
		 rwsim::dynamics::Body::Ptr getBody1() const { return _body; }

		/**
		 * @brief Get the second body (the sensor body).
		 * @return the Body.
		 */
		 rwsim::dynamics::Body::Ptr getBody2() const { return _body1; }

	private:
		SimulatedFTSensor();

	private:
		rwsim::dynamics::Body::Ptr _body, _body1;
		rw::kinematics::Frame *_sframe;
		rw::common::Ptr<rw::sensor::FTSensorModel> _ftmodel;

		struct FTStateData {
			rw::math::Vector3D<> _force, _forceTmp, _torque, _torqueTmp;

			//! aux variables updated through \b update
			rw::math::Transform3D<> _fTb, _wTb, _bTw;
		};

		 rw::kinematics::StatelessData< FTStateData > _sdata;
	};
	//! @}
}
}

#endif /* RWSIM_SENSOR_SIMULATEDFTSENSOR_HPP_ */
