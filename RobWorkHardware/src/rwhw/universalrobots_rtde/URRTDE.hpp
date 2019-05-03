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

class RTDEControlInterface;
class RTDEReceiveInterface;

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

            void stopRobot();
            bool reuploadScript();

            bool moveJ(const rw::math::Q& q, double speed, double acceleration);
            bool moveJ(const rw::trajectory::QPath& q_path);
            bool moveJ_IK(const rw::math::Transform3D<> &pose, double speed, double acceleration);

            bool moveL(const rw::math::Transform3D<>& pose, double speed, double acceleration);
            bool moveL(const rw::trajectory::Transform3DPath& pose_path);
            bool moveL_FK(const rw::math::Q& q, double speed, double acceleration);

            bool moveC(const rw::math::Transform3D<>& pose_via, const rw::math::Transform3D<>& pose_to,
                        double speed, double acceleration);

            bool speedJ(const rw::math::Q& qd, double acceleration, double time = 0.0);
            bool speedL(const rw::math::Q& xd, double acceleration, double time = 0.0);

            bool servoJ(const rw::math::Q& q, double speed, double acceleration,
                        double time, double lookahead_time, double gain);

            bool servoStop();

            bool servoC(const rw::math::Transform3D<>& pose, double speed, double acceleration, double blend);

            bool forceModeStart(const rw::math::Transform3D<>& task_frame, const rw::math::Q& selection_vector,
                                const rw::math::Wrench6D<>& wrench, int type, const rw::math::Q& limits);

            bool forceModeUpdate(const rw::math::Wrench6D<>& wrench);

            bool forceModeStop();

            bool zeroFtSensor();

            bool setStandardDigitalOut(std::uint8_t output_id, bool signal_level);

            bool setToolDigitalOut(std::uint8_t output_id, bool signal_level);

            // Receive interface functions

            /**
              * @returns Time elapsed since the controller was started [s]
              */
            double getTimestamp();

            /**
              * @returns Target joint positions
              */
            rw::math::Q getTargetQ();

            /**
              * @returns Target joint velocities
              */
            rw::math::Q getTargetQd();

            /**
              * @returns Target joint accelerations
              */
            rw::math::Q getTargetQdd();

            /**
              * @returns Target joint currents
              */
            rw::math::Q getTargetCurrent();

            /**
              * @returns Target joint moments (torques)
              */
            rw::math::Q getTargetMoment();

            /**
              * @returns Actual joint positions
              */
            rw::math::Q getActualQ();

            rw::math::Q getActualQd();

            rw::math::Q getActualCurrent();

            rw::math::Q getJointControlOutput();

            rw::math::Transform3D<> getActualTCPPose();

            rw::math::Q getActualTCPSpeed();

            rw::math::Wrench6D<> getActualTCPForce();

            rw::math::Transform3D<> getTargetTCPPose();

            rw::math::Q getTargetTCPSpeed();

            uint64_t getActualDigitalInputBits();

            rw::math::Q getJointTemperatures();

            double getActualExecutionTime();

            int32_t getRobotMode();

            std::vector<int32_t> getJointMode();

            int32_t getSafetyMode();

            rw::math::Q getActualToolAccelerometer();

            double getSpeedScaling();

            double getTargetSpeedFraction();

            double getActualMomentum();

            double getActualMainVoltage();

            double getActualRobotVoltage();

            double getActualRobotCurrent();

            rw::math::Q getActualJointVoltage();

            uint64_t getActualDigitalOutputBits();

            uint32_t getRuntimeState();

        private:
            std::shared_ptr<RTDEControlInterface> rtde_control_;
            std::shared_ptr<RTDEReceiveInterface> rtde_receive_;
    };
//! @}

} /* namespace rwhw */

#endif /* RWHW_UNIVERSALROBOTS_RTDE_URRTDE_HPP_ */
