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

#ifndef RWSIM_UTIL_RECURSIVENEWTONEULER_HPP_
#define RWSIM_UTIL_RECURSIVENEWTONEULER_HPP_

/**
 * @file RecursiveNewtonEuler.hpp
 *
 * \copydoc rwsim::util::RecursiveNewtonEuler
 */

#include <rw/common/Ptr.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Wrench6D.hpp>
#include <rw/math/VelocityScrew6D.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>

#include <boost/thread/mutex.hpp>

#include <vector>

namespace rwsim {
namespace util {
//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief The recursive Newton-Euler method is used for calculating inverse dynamics of a kinematic tree.
 *
 * Given the motion of a robot (that is \f$\bf{q}\f$, \f$\bf{\dot{q}}\f$, and \f$\bf{\ddot{q}}\f$),
 * this class can calculate the forces and torques at the joints which will cause the desired motion.
 *
 * Furthermore it is possible to get information about the motion of the individual bodies in their center of mass,
 * as well as the required net forces on these bodies to achieve the desired motion (also in the center of mass).
 *
 * @note Currently this class supports only devices of type SerialDevice with revolute or prismatic joints.
 * If this constraint is violated, validate() will thrown an exception.
 */
class RecursiveNewtonEuler {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<RecursiveNewtonEuler> Ptr;

	/**
	 * @brief Constructor for inverse dynamics for a RigidDevice.
	 * @note Currently this class supports only devices of type SerialDevice.
	 * @param device [in] a rigid device.
	 */
	RecursiveNewtonEuler(rwsim::dynamics::RigidDevice::Ptr device);

	//! @brief Destructor.
	virtual ~RecursiveNewtonEuler();

	/**
	 * @brief Get the gravity acting on the links.
	 * @return gravity vector in base coordinates.
	 */
	rw::math::Vector3D<> getGravity() const;

	/**
	 * @brief Set the gravity acting on the links.
	 * @param gravity [in] gravity vector in base coordinates.
	 */
	void setGravity(const rw::math::Vector3D<> &gravity);

	/**
	 * @brief Get the center of mass of the payload in the end frame.
	 * @return center of mass of the payload, \f$\overline{{\bf{\Delta r}}_{tool}^{com}}\f$.
	 */
	rw::math::Vector3D<> getPayloadCOM() const;

	/**
	 * @brief Get the mass of the payload, \f$m_{tool}\f$.
	 * @return mass of the payload.
	 */
	double getPayloadMass() const;

	/**
	 * @brief Get the inertia of the payload, \f$\overline{\bf{I_{tool}}}\f$.
	 * @return inertia of payload in end frame of device.
	 */
	rw::math::InertiaMatrix<> getPayloadInertia() const;

	/**
	 * @brief Set the payload relative to the end frame of the device.
	 *
	 * Note that also the inertia is defined locally in the end frame.
	 *
	 * @param com [in] the center of mass in the end frame, \f$\overline{{\bf{\Delta r}}_{tool}^{com}}\f$.
	 * @param payload [in] the mass of the payload, \f$m_{tool}\f$.
	 * @param inertia [in] the inertia of the payload, \f$\overline{\bf{I_{tool}}}\f$.
	 */
	void setPayload(const rw::math::Vector3D<> &com, double payload, const rw::math::InertiaMatrix<> &inertia);

	/**
	 * @brief Get the force and torque that the robot exerts on the environment at the TCP frame in base coordinates.
	 * @return a 6D wrench
	 */
	rw::math::Wrench6D<> getEnvironment() const;

	/**
	 * @brief Set the force and torque that the robot exerts on the environment at the TCP frame in base coordinates.
	 * @param wrench [in] the wrench the robot exerts on the environment
	 */
	void setEnvironment(const rw::math::Wrench6D<> &wrench);

	/**
	 * @brief Motion of a body defined as velocity and acceleration.
	 *
	 * The motion is given in base coordinates with origin in the center of mass.
	 */
	struct Motion {
		//! @brief Linear and angular velocity of the body i (\f$\bf{\dot{r}_i^{com}}\f$ and \f$\bf{\omega_i}\f$)
		rw::math::VelocityScrew6D<> velocity;
		//! @brief Linear and angular acceleration of the body i (\f$\bf{\ddot{r}_i^{com}}\f$ and \f$\bf{\dot{\omega}_i}\f$)
		rw::math::VelocityScrew6D<> acceleration;
	};

	/**
	 * @brief Get the Motion of each link and the motion of the payload for a device that have moving joints.
	 *
	 * The Motion is given as the linear and angular velocity and acceleration of each body, i.
	 * (\f$\bf{\dot{r}_i^{com}}\f$, \f$\bf{\ddot{r}_i^{com}}\f$, \f$\bf{\omega_i}\f$, and \f$\bf{\dot{\omega}_i}\f$)
	 *
	 * An additional motion is appended to the list of motions, namely the motion of the payload:
	 * (\f$\bf{\dot{r}_{tool}^{com}}\f$, \f$\bf{\ddot{r}_{tool}^{com}}\f$, \f$\bf{\omega_{tool}}\f$, and \f$\bf{\dot{\omega}_{tool}}\f$)
	 *
	 * @param state [in] the state of the device (\f$\bf{q}\f$ is extracted from this)
	 * @param dq [in] joint speeds, \f$\bf{\dot{q}}\f$.
	 * @param ddq [in] joint accelerations, \f$\bf{\ddot{q}}\f$.
	 * @return vector of Motion values with a length one greater than the degrees of freedom of the robot.
	 */
	std::vector<Motion> getBodyMotion(const rw::kinematics::State &state, const rw::math::Q &dq, const rw::math::Q &ddq) const;

	/**
	 * @brief Determine the net force and torque acting on each link to achieve the given motion.
	 * @param motions [in] the motion of each link (possibly retrieved from getBodyMotion()).
	 * @param state [in] state (position) of the robot.
	 * @return vector of Wrench6D values with a length one greater than the degrees of freedom of the robot.
	 */
	std::vector<rw::math::Wrench6D<> > getBodyNetForces(const std::vector<Motion> &motions, const rw::kinematics::State &state) const;

	/**
	 * @brief Determine the forces and torques acting in each joint to achieve the given net forces and torques on the links and payload.
	 *
	 * The joint force and torque is given in base coordinates, but acting in the joint:
	 * (\f$\bf{f}_i\f$ and \f$\bf{n}_i\f$)
	 *
	 * @param bodyNetForces [in] the desired net forces and torques acting on each link in link local coordinate frames (possibly retrieved from getBodyNetForces()).
	 * @param state [in] state (position) of the robot.
	 * @return vector of Wrench6D values - one for each joint.
	 */
	std::vector<rw::math::Wrench6D<> > getJointForces(const std::vector<rw::math::Wrench6D<> > &bodyNetForces, const rw::kinematics::State &state) const;

	/**
	 * @brief Do inverse dynamics for a robot in motion.
	 *
	 * The joint force and torque is given in base coordinates, but acting in the joint:
	 * (\f$\bf{f}_i\f$ and \f$\bf{n}_i\f$)
	 *
	 * @param state [in] the state of the device (\f$\bf{q}\f$ is extracted from this)
	 * @param dq [in] joint speeds, \f$\bf{\dot{q}}\f$.
	 * @param ddq [in] joint accelerations, \f$\bf{\ddot{q}}\f$.
	 * @return the wrenches working in each joint in base coordinates
	 */
	std::vector<rw::math::Wrench6D<> > solve(const rw::kinematics::State &state, const rw::math::Q dq = rw::math::Q(), const rw::math::Q ddq = rw::math::Q()) const;

	/**
	 * @brief Get the torque provided by each motor, \f$\tau_i\f$.
	 * @param state [in] the state of the device (\f$\bf{q}\f$ is extracted from this)
	 * @param dq [in] joint speeds, \f$\bf{\dot{q}}\f$.
	 * @param ddq [in] joint accelerations, \f$\bf{\ddot{q}}\f$.
	 * @return a Q vector of the torques provided by each motor
	 */
	rw::math::Q solveMotorTorques(const rw::kinematics::State &state, const rw::math::Q &dq, const rw::math::Q &ddq) const;

	/**
	 * @brief Check if solver will work for the given device - if not an exception will be thrown when trying to solve.
	 * @return true if solver works for the given device, false otherwise.
	 */
	bool validate() const;

private:
	static std::string invalidMsg();
	static rw::math::Vector3D<> toVector3D(const rw::math::EAA<> &eaa);

	const rw::common::Ptr<rwsim::dynamics::RigidDevice> _rdev;
	const rw::common::Ptr<const rw::models::JointDevice> _jdev;
	const bool _valid;

	rw::math::Vector3D<> _gravity;
	rw::math::Vector3D<> _payloadCOM;
	double _payloadMass;
	rw::math::InertiaMatrix<> _payloadInertia;
	rw::math::Wrench6D<> _environment;
	mutable boost::mutex _mutex;
};
//! @}
} /* namespace util */
} /* namespace rwsim */
#endif /* RWSIM_UTIL_RECURSIVENEWTONEULER_HPP_ */
