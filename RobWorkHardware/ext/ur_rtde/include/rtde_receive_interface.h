#ifndef RTDE_RECEIVE_INTERFACE_H
#define RTDE_RECEIVE_INTERFACE_H

#include <rtde_export.h>
#include <rtde.h>
#include <dashboard_client.h>
#include <script_client.h>
#include <thread>
#include <future>
#include <chrono>
#include <sstream>
#include <iostream>
#include <boost/thread.hpp>

#define MAJOR_VERSION 0
#define CB3_MAJOR_VERSION 3

class RTDEReceiveInterface
{
 public:

  RTDE_EXPORT explicit RTDEReceiveInterface(std::string hostname, std::vector<std::string> variables = {},
                                            int port = 30004);

  RTDE_EXPORT virtual ~RTDEReceiveInterface();

  /**
    * @returns Time elapsed since the controller was started [s]
    */
  RTDE_EXPORT double getTimestamp();

  /**
    * @returns Target joint positions
    */
  RTDE_EXPORT std::vector<double> getTargetQ();

  /**
    * @returns Target joint velocities
    */
  RTDE_EXPORT std::vector<double> getTargetQd();

  /**
    * @returns Target joint accelerations
    */
  RTDE_EXPORT std::vector<double> getTargetQdd();

  /**
    * @returns Target joint currents
    */
  RTDE_EXPORT std::vector<double> getTargetCurrent();

  /**
    * @returns Target joint moments (torques)
    */
  RTDE_EXPORT std::vector<double> getTargetMoment();

  /**
    * @returns Actual joint positions
    */
  RTDE_EXPORT std::vector<double> getActualQ();

  RTDE_EXPORT std::vector<double> getActualQd();

  RTDE_EXPORT std::vector<double> getActualCurrent();

  RTDE_EXPORT std::vector<double> getJointControlOutput();

  RTDE_EXPORT std::vector<double> getActualTCPPose();

  RTDE_EXPORT std::vector<double> getActualTCPSpeed();

  RTDE_EXPORT std::vector<double> getActualTCPForce();

  RTDE_EXPORT std::vector<double> getTargetTCPPose();

  RTDE_EXPORT std::vector<double> getTargetTCPSpeed();

  RTDE_EXPORT uint64_t getActualDigitalInputBits();

  RTDE_EXPORT std::vector<double> getJointTemperatures();

  RTDE_EXPORT double getActualExecutionTime();

  RTDE_EXPORT int32_t getRobotMode();

  RTDE_EXPORT std::vector<int32_t> getJointMode();

  RTDE_EXPORT int32_t getSafetyMode();

  RTDE_EXPORT std::vector<double> getActualToolAccelerometer();

  RTDE_EXPORT double getSpeedScaling();

  RTDE_EXPORT double getTargetSpeedFraction();

  RTDE_EXPORT double getActualMomentum();

  RTDE_EXPORT double getActualMainVoltage();

  RTDE_EXPORT double getActualRobotVoltage();

  RTDE_EXPORT double getActualRobotCurrent();

  RTDE_EXPORT std::vector<double> getActualJointVoltage();

  RTDE_EXPORT uint64_t getActualDigitalOutputBits();

  RTDE_EXPORT uint32_t getRuntimeState();

  RTDE_EXPORT void receiveCallback();

 private:
  std::vector<std::string> variables_;
  std::string hostname_;
  int port_;
  std::shared_ptr<RTDE> rtde_;
  std::atomic<bool> stop_thread {false};
  std::shared_ptr<boost::thread> th_;
  std::shared_ptr<RobotState> robot_state_;
};

#endif  // RTDE_RECEIVE_INTERFACE_H
