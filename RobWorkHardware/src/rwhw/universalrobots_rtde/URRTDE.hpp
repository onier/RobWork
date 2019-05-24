/******************************************************************************
 * Copyright 2019 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
 ******************************************************************************/

#ifndef RWHW_UNIVERSALROBOTS_RTDE_URRTDE_HPP_
#define RWHW_UNIVERSALROBOTS_RTDE_URRTDE_HPP_

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Wrench6D.hpp>
#include <rw/trajectory/Path.hpp>
#include <string>
#include <vector>

namespace ur_rtde {
    class RTDEControlInterface;
    class RTDEReceiveInterface;
}

/**
 * @file URRTDE.hpp
 *
 * \copydoc rwhw::URRTDE
 */

namespace rwhw {

    //! @addtogroup rwhw

    //! @{
    /**
     * @brief URRTDE class provides a control interface for the UR Robots using RTDE
     */
    class URRTDE
    {
        public:
            explicit URRTDE(std::string robot_ip, std::vector<std::string> variables = {});
            virtual ~URRTDE();

            // Control interface functions

            //! @copydoc ur_rtde::RTDEControlInterface::stopRobot
            void stopRobot();

            //! @copydoc ur_rtde::RTDEControlInterface::reuploadScript
            bool reuploadScript();

            /**
             * @brief Move to joint configuration (linear in joint-space)
             * @param q joint positions
             * @param speed joint speed of leading axis [rad/s]
             * @param acceleration joint acceleration of leading axis [rad/s^2]
             */
            bool moveJ(const rw::math::Q& q, double speed, double acceleration);

            /**
             * @brief Move to each joint configuration specified in a QPath
             * @param q_path with joint configurations that includes acceleration, speed and blend for each configuration
             */
            bool moveJ(const rw::trajectory::QPath& q_path);

            /**
             * @brief Move to pose (linear in joint-space)
             * @param pose target pose as Transform3D
             * @param speed joint speed of leading axis [rad/s]
             * @param acceleration joint acceleration of leading axis [rad/s^2]
             */
            bool moveJ_IK(const rw::math::Transform3D<> &pose, double speed, double acceleration);

            /**
             * @brief Move to position (linear in tool-space)
             * @param pose target pose
             * @param speed tool speed [m/s]
             * @param acceleration tool acceleration [m/s^2]
             */
            bool moveL(const rw::math::Transform3D<>& pose, double speed, double acceleration);

            /**
             * @brief Move to each pose specified in a Transform3DPath
             * @param path with tool poses specified as Transform3D
             */
            bool moveL(const rw::trajectory::Transform3DPath& pose_path);

            /**
             * @brief Move to position (linear in tool-space)
             * @param q joint positions
             * @param speed tool speed [m/s]
             * @param acceleration tool acceleration [m/s^2]
             */
            bool moveL_FK(const rw::math::Q& q, double speed, double acceleration);

            /**
             * @brief Joint speed - Accelerate linearly in joint space and continue with constant joint speed
             * @param qd joint speeds [rad/s]
             * @param acceleration joint acceleration [rad/s^2] (of leading axis)
             * @param time time [s] before the function returns (optional)
             */
            bool speedJ(const rw::math::Q& qd, double acceleration, double time = 0.0);

            /**
             * @brief Tool speed - Accelerate linearly in Cartesian space and continue with constant tool speed. The time t is
             * optional;
             * @param xd tool speed [m/s] (spatial vector)
             * @param acceleration tool position acceleration [m/s^2]
             * @param time time [s] before the function returns (optional)
             */
            bool speedL(const rw::math::Q& xd, double acceleration, double time = 0.0);

            /**
             * @brief Servo to position (linear in joint-space)
             * @param q joint positions [rad]
             * @param speed NOT used in current version
             * @param acceleration NOT used in current version
             * @param time time where the command is controlling the robot. The function is blocking for time t [S]
             * @param lookahead_time time [S], range [0.03,0.2] smoothens the trajectory with this lookahead time
             * @param gain proportional gain for following target position, range [100,2000]
             */
            bool servoJ(const rw::math::Q& q, double speed, double acceleration,
                        double time, double lookahead_time, double gain);

            //! @copydoc ur_rtde::RTDEControlInterface::servoStop
            bool servoStop();

            //! @copydoc ur_rtde::RTDEControlInterface::speedStop
            bool speedStop();

            /**
             * @brief Set robot to be controlled in force mode
             * @param task_frame A Transform3D pose that defines the force frame relative to the base frame.
             * @param selection_vector A 6d Q vector of 0s and 1s. 1 means that the robot will be compliant in the corresponding
             * axis of the task frame
             * @param wrench The forces/torques the robot will apply to its environment. The robot adjusts its position
             * along/about compliant axis in order to achieve the specified force/torque. Values have no effect for
             * non-compliant axes
             * @param type An integer [1;3] specifying how the robot interprets the force frame.
             * 1: The force frame is transformed in a way such that its y-axis is aligned with a vector pointing from the
             * robot tcp towards the origin of the force frame. 2: The force frame is not transformed. 3: The force frame is
             * transformed in a way such that its x-axis is the projection of the robot tcp velocity vector onto the x-y plane
             * of the force frame.
             * @param limits (Float) 6d Q vector. For compliant axes, these values are the maximum allowed tcp speed along/about
             * the axis. For non-compliant axes, these values are the maximum allowed deviation along/about an axis between the
             * actual tcp position and the one set by the program.
             */
            bool forceModeStart(const rw::math::Transform3D<>& task_frame, const rw::math::Q& selection_vector,
                                const rw::math::Wrench6D<>& wrench, int type, const rw::math::Q& limits);

            /**
             * @brief Update the wrench the robot will apply to its environment
             * @param wrench The forces/torques the robot will apply to its environment. The robot adjusts its position
             * along/about compliant axis in order to achieve the specified force/torque. Values have no effect for
             * non-compliant axes
             */
            bool forceModeUpdate(const rw::math::Wrench6D<>& wrench);

            //! @copydoc ur_rtde::RTDEControlInterface::forceModeStop
            bool forceModeStop();

            //! @copydoc ur_rtde::RTDEControlInterface::zeroFtSensor
            bool zeroFtSensor();

            //! @copydoc ur_rtde::RTDEControlInterface::setStandardDigitalOut
            bool setStandardDigitalOut(std::uint8_t output_id, bool signal_level);

            //! @copydoc ur_rtde::RTDEControlInterface::setToolDigitalOut
            bool setToolDigitalOut(std::uint8_t output_id, bool signal_level);

            //! @copydoc ur_rtde::RTDEControlInterface::setPayload
            bool setPayload(double mass, const std::vector<double> &cog = {});

            //! @copydoc ur_rtde::RTDEControlInterface::teachMode
            bool teachMode();

            //! @copydoc ur_rtde::RTDEControlInterface::endTeachMode
            bool endTeachMode();

            //! @copydoc ur_rtde::RTDEControlInterface::forceModeSetDamping
            bool forceModeSetDamping(double damping);

            //! @copydoc ur_rtde::RTDEControlInterface::forceModeSetGainScaling
            bool forceModeSetGainScaling(double scaling);

            //! @copydoc ur_rtde::RTDEControlInterface::setSpeedSlider
            bool setSpeedSlider(double speed);

            //! @copydoc ur_rtde::RTDEControlInterface::setAnalogOutputVoltage
            bool setAnalogOutputVoltage(std::uint8_t output_id, double voltage_ratio);

            //! @copydoc ur_rtde::RTDEControlInterface::setAnalogOutputCurrent
            bool setAnalogOutputCurrent(std::uint8_t output_id, double current_ratio);

            //! @copydoc ur_rtde::RTDEControlInterface::isProgramRunning
            bool isProgramRunning();

            // Receive interface functions

            //! @copydoc ur_rtde::RTDEReceiveInterface::getTimestamp
            double getTimestamp();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getTargetQ
            rw::math::Q getTargetQ();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getTargetQd
            rw::math::Q getTargetQd();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getTargetQdd
            rw::math::Q getTargetQdd();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getTargetCurrent
            rw::math::Q getTargetCurrent();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getTargetMoment
            rw::math::Q getTargetMoment();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getActualQ
            rw::math::Q getActualQ();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getActualQd
            rw::math::Q getActualQd();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getActualCurrent
            rw::math::Q getActualCurrent();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getJointControlOutput
            rw::math::Q getJointControlOutput();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getActualTCPPose
            rw::math::Transform3D<> getActualTCPPose();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getActualTCPSpeed
            rw::math::Q getActualTCPSpeed();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getActualTCPForce
            rw::math::Wrench6D<> getActualTCPForce();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getTargetTCPPose
            rw::math::Transform3D<> getTargetTCPPose();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getTargetTCPSpeed
            rw::math::Q getTargetTCPSpeed();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getActualDigitalInputBits
            uint64_t getActualDigitalInputBits();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getJointTemperatures
            rw::math::Q getJointTemperatures();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getActualExecutionTime
            double getActualExecutionTime();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getRobotMode
            int32_t getRobotMode();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getRobotStatus
            uint32_t getRobotStatus();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getRobotStatus
            std::vector<int32_t> getJointMode();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getSafetyMode
            int32_t getSafetyMode();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getActualToolAccelerometer
            rw::math::Q getActualToolAccelerometer();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getSpeedScaling
            double getSpeedScaling();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getTargetSpeedFraction
            double getTargetSpeedFraction();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getActualMomentum
            double getActualMomentum();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getActualMainVoltage
            double getActualMainVoltage();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getActualRobotVoltage
            double getActualRobotVoltage();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getActualRobotCurrent
            double getActualRobotCurrent();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getActualJointVoltage
            rw::math::Q getActualJointVoltage();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getActualDigitalOutputBits
            uint64_t getActualDigitalOutputBits();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getRuntimeState
            uint32_t getRuntimeState();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getStandardAnalogInput0
            double getStandardAnalogInput0();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getStandardAnalogInput1
            double getStandardAnalogInput1();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getStandardAnalogOutput0
            double getStandardAnalogOutput0();

            //! @copydoc ur_rtde::RTDEReceiveInterface::getStandardAnalogOutput1
            double getStandardAnalogOutput1();

        private:
            std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control_;
            std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_;
    };
//! @}

} /* namespace rwhw */

#endif /* RWHW_UNIVERSALROBOTS_RTDE_URRTDE_HPP_ */
