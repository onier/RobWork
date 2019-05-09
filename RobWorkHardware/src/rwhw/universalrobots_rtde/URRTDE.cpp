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

#include "URRTDE.hpp"
#include <rtde_control_interface.h>
#include <rtde_receive_interface.h>

using namespace rwhw;

URRTDE::URRTDE(std::string robot_ip, std::vector<std::string> variables)
{
    // Initialize interface
    rtde_control_ = std::make_shared<RTDEControlInterface>(robot_ip);
    rtde_receive_ = std::make_shared<RTDEReceiveInterface>(robot_ip, variables);
}

URRTDE::~URRTDE()
{
}

bool URRTDE::reuploadScript() {
    return rtde_control_->reuploadScript();
}

void URRTDE::stopRobot() {
    rtde_control_->stopRobot();
}

bool URRTDE::moveJ(const rw::math::Q& q, double speed, double acceleration) {
    return rtde_control_->moveJ(q.toStdVector(), speed, acceleration);
}

bool URRTDE::moveJ(const rw::trajectory::QPath& q_path) {
    std::vector<std::vector<double>> path;
    for(const auto &q : q_path)
        path.push_back(q.toStdVector());
    return rtde_control_->moveJ(path);
}

bool URRTDE::moveJ_IK(const rw::math::Transform3D<> &pose, double speed, double acceleration) {
    rw::math::Vector3D<> pos = pose.P();
    rw::math::EAA<> eaa(pose.R());
    std::vector<double> std_pose = {pos[0], pos[1], pos[2], eaa[0], eaa[1], eaa[2]};
    return rtde_control_->moveJ_IK(std_pose, speed, acceleration);
}

bool URRTDE::moveL(const rw::math::Transform3D<> &pose, double speed, double acceleration) {
    rw::math::Vector3D<> pos = pose.P();
    rw::math::EAA<> eaa(pose.R());
    std::vector<double> std_pose = {pos[0], pos[1], pos[2], eaa[0], eaa[1], eaa[2]};
    return rtde_control_->moveL(std_pose, speed, acceleration);
}

bool URRTDE::moveL(const rw::trajectory::Transform3DPath& pose_path) {
    std::vector<std::vector<double>> path;
    for(const auto &transform : pose_path)
    {
        rw::math::Vector3D<> pos = transform.P();
        rw::math::EAA<> eaa(transform.R());
        // TODO: Currently speed and acceleration and blend are hardcoded for this mode.
        std::vector<double> std_pose = {pos[0], pos[1], pos[2], eaa[0], eaa[1], eaa[2], 0.6, 1.2, 0};
        path.push_back(std_pose);
    }
    return rtde_control_->moveL(path);
}

bool URRTDE::moveL_FK(const rw::math::Q& q, double speed, double acceleration) {
    return rtde_control_->moveL_FK(q.toStdVector(), speed, acceleration);
}

bool URRTDE::speedJ(const rw::math::Q& qd, double acceleration, double time) {
    return rtde_control_->speedJ(qd.toStdVector(), acceleration, time);
}

bool URRTDE::speedL(const rw::math::Q& xd, double acceleration, double time) {
    return rtde_control_->speedL(xd.toStdVector(), acceleration, time);
}

bool URRTDE::servoJ(const rw::math::Q& q, double speed, double acceleration, double time, double lookahead_time,
                    double gain) {
    return rtde_control_->servoJ(q.toStdVector(), speed, acceleration, time, lookahead_time, gain);
}

bool URRTDE::servoStop() {
    return rtde_control_->servoStop();
}

bool URRTDE::forceModeStart(const rw::math::Transform3D<>& task_frame, const rw::math::Q& selection_vector,
                            const rw::math::Wrench6D<>& wrench, int type, const rw::math::Q& limits) {
    rw::math::Vector3D<> pos = task_frame.P();
    rw::math::EAA<> eaa(task_frame.R());
    std::vector<double> std_task_frame = {pos[0], pos[1], pos[2], eaa[0], eaa[1], eaa[2]};
    std::vector<double> std_selection_vector_double = selection_vector.toStdVector();
    std::vector<int> std_selection_vector(std_selection_vector_double.begin(), std_selection_vector_double.end());
    const rw::math::Vector3D<> force = wrench.force();
    const rw::math::Vector3D<> torque = wrench.torque();
    std::vector<double> std_wrench = {force[0], force[1], force[2], torque[0], torque[1], torque[2]};

    return rtde_control_->forceModeStart(std_task_frame, std_selection_vector, std_wrench, type,
                                         limits.toStdVector());
}

bool URRTDE::forceModeUpdate(const rw::math::Wrench6D<>& wrench) {
    const rw::math::Vector3D<> force = wrench.force();
    const rw::math::Vector3D<> torque = wrench.torque();
    std::vector<double> std_wrench = {force[0], force[1], force[2], torque[0], torque[1], torque[2]};
    return rtde_control_->forceModeUpdate(std_wrench);
}

bool URRTDE::forceModeStop() {
    return rtde_control_->forceModeStop();
}

bool URRTDE::zeroFtSensor() {
    return rtde_control_->zeroFtSensor();
}

bool URRTDE::setStandardDigitalOut(std::uint8_t output_id, bool signal_level) {
    return rtde_control_->setStandardDigitalOut(output_id, signal_level);
}

bool URRTDE::setToolDigitalOut(std::uint8_t output_id, bool signal_level) {
    return rtde_control_->setToolDigitalOut(output_id, signal_level);
}

double URRTDE::getTimestamp() {
    return rtde_receive_->getTimestamp();
}

rw::math::Q URRTDE::getTargetQ() {
    return rw::math::Q(rtde_receive_->getTargetQ());
}

rw::math::Q URRTDE::getTargetQd() {
    return rw::math::Q(rtde_receive_->getTargetQd());
}

rw::math::Q URRTDE::getTargetQdd() {
    return rw::math::Q(rtde_receive_->getTargetQdd());
}

rw::math::Q URRTDE::getTargetCurrent() {
    return rw::math::Q(rtde_receive_->getTargetCurrent());
}

rw::math::Q URRTDE::getTargetMoment() {
    return rw::math::Q(rtde_receive_->getTargetMoment());
}

rw::math::Q URRTDE::getActualQ() {
    return rw::math::Q(rtde_receive_->getActualQ());
}

rw::math::Q URRTDE::getActualQd() {
    return rw::math::Q(rtde_receive_->getActualQd());
}

rw::math::Q URRTDE::getActualCurrent() {
    return rw::math::Q(rtde_receive_->getActualCurrent());
}

rw::math::Q URRTDE::getJointControlOutput() {
    return rw::math::Q(rtde_receive_->getJointControlOutput());
}

rw::math::Transform3D<> URRTDE::getActualTCPPose() {
    const std::vector<double> std_pose = rtde_receive_->getActualTCPPose();
    rw::math::Vector3D<> pos(std_pose[0], std_pose[1], std_pose[2]);
    rw::math::EAA<> eaa(std_pose[3], std_pose[4], std_pose[5]);
    rw::math::Transform3D<> tcp_pose(pos, eaa.toRotation3D());
    return tcp_pose;
}

rw::math::Q URRTDE::getActualTCPSpeed() {
    return rw::math::Q(rtde_receive_->getActualTCPSpeed());
}

rw::math::Wrench6D<> URRTDE::getActualTCPForce() {
    const std::vector<double> std_w = rtde_receive_->getActualTCPForce();
    rw::math::Wrench6D<> wrench(std_w[0], std_w[1], std_w[2], std_w[3], std_w[4], std_w[5]);
    return wrench;
}

rw::math::Transform3D<> URRTDE::getTargetTCPPose() {
    const std::vector<double> std_pose = rtde_receive_->getTargetTCPPose();
    rw::math::Vector3D<> pos(std_pose[0], std_pose[1], std_pose[2]);
    rw::math::EAA<> eaa(std_pose[3], std_pose[4], std_pose[5]);
    rw::math::Transform3D<> tcp_pose(pos, eaa.toRotation3D());
    return tcp_pose;
}

rw::math::Q URRTDE::getTargetTCPSpeed() {
    return rw::math::Q(rtde_receive_->getTargetTCPSpeed());
}

uint64_t URRTDE::getActualDigitalInputBits() {
    return rtde_receive_->getActualDigitalInputBits();
}

rw::math::Q URRTDE::getJointTemperatures() {
    return rw::math::Q(rtde_receive_->getJointTemperatures());
}

double URRTDE::getActualExecutionTime() {
    return rtde_receive_->getActualExecutionTime();
}

int32_t URRTDE::getRobotMode() {
    return rtde_receive_->getRobotMode();
}

std::vector<int32_t> URRTDE::getJointMode() {
    return rtde_receive_->getJointMode();
}

int32_t URRTDE::getSafetyMode() {
    return rtde_receive_->getSafetyMode();
}

rw::math::Q URRTDE::getActualToolAccelerometer() {
    return rw::math::Q(rtde_receive_->getActualToolAccelerometer());
}

double URRTDE::getSpeedScaling() {
    return rtde_receive_->getSpeedScaling();
}

double URRTDE::getTargetSpeedFraction() {
    return rtde_receive_->getTargetSpeedFraction();
}

double URRTDE::getActualMomentum() {
    return rtde_receive_->getActualMomentum();
}

double URRTDE::getActualMainVoltage() {
    return rtde_receive_->getActualMainVoltage();
}

double URRTDE::getActualRobotVoltage() {
    return rtde_receive_->getActualRobotVoltage();
}

double URRTDE::getActualRobotCurrent() {
    return rtde_receive_->getActualRobotCurrent();
}

rw::math::Q URRTDE::getActualJointVoltage() {
    return rw::math::Q(rtde_receive_->getActualJointVoltage());
}

uint64_t URRTDE::getActualDigitalOutputBits() {
    return rtde_receive_->getActualDigitalOutputBits();
}

uint32_t URRTDE::getRuntimeState() {
    return rtde_receive_->getRuntimeState();
}


