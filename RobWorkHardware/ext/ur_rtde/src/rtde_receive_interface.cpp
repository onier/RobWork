#include <ur_rtde/rtde_receive_interface.h>
#include <iostream>

namespace ur_rtde
{
RTDEReceiveInterface::RTDEReceiveInterface(std::string hostname, std::vector<std::string> variables, int port)
    : variables_(std::move(variables)), hostname_(std::move(hostname)), port_(port)
{
  rtde_ = std::make_shared<RTDE>(hostname_);
  rtde_->connect();
  rtde_->negotiateProtocolVersion();
  auto controller_version = rtde_->getControllerVersion();
  uint32_t major_version = std::get<MAJOR_VERSION>(controller_version);

  double frequency = 125;
  // If e-Series Robot set frequency to 500Hz
  if (major_version > CB3_MAJOR_VERSION)
    frequency = 500;

  if (variables_.empty())
  {
    // Assume all variables
    variables_ = {
        "timestamp",              "target_q",                   "target_qd",              "target_qdd",
        "target_current",         "target_moment",              "actual_q",               "actual_qd",
        "actual_current",         "joint_control_output",       "actual_TCP_pose",        "actual_TCP_speed",
        "actual_TCP_force",       "target_TCP_pose",            "target_TCP_speed",       "actual_digital_input_bits",
        "joint_temperatures",     "actual_execution_time",      "robot_mode",             "joint_mode",
        "safety_mode",            "actual_tool_accelerometer",  "speed_scaling",          "target_speed_fraction",
        "actual_momentum",        "actual_main_voltage",        "actual_robot_voltage",   "actual_robot_current",
        "actual_joint_voltage",   "actual_digital_output_bits", "runtime_state",          "standard_analog_input0",
        "standard_analog_input0", "standard_analog_output0",    "standard_analog_output1", "robot_status_bits"};
  }

  // Setup output
  rtde_->sendOutputSetup(variables_, frequency);

  // Start RTDE data synchronization
  rtde_->sendStart();

  // Init Robot state
  robot_state_ = std::make_shared<RobotState>();

  // Start executing receiveCallback
  th_ = std::make_shared<boost::thread>(boost::bind(&RTDEReceiveInterface::receiveCallback, this));

  // Wait until the first robot state has been received
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

RTDEReceiveInterface::~RTDEReceiveInterface()
{
  if (rtde_ != nullptr)
  {
    if (rtde_->isConnected())
      rtde_->disconnect();
  }

  // Stop the receive callback function
  stop_thread = true;
  th_->interrupt();
  th_->join();
}

void RTDEReceiveInterface::receiveCallback()
{
  while (!stop_thread)
  {
    // Receive and update the robot state
    rtde_->receiveData(robot_state_);
  }
}

double RTDEReceiveInterface::getTimestamp()
{
  return robot_state_->getTimestamp();
}

std::vector<double> RTDEReceiveInterface::getTargetQ()
{
  return robot_state_->getTarget_q();
}

std::vector<double> RTDEReceiveInterface::getTargetQd()
{
  return robot_state_->getTarget_qd();
}

std::vector<double> RTDEReceiveInterface::getTargetQdd()
{
  return robot_state_->getTarget_qdd();
}

std::vector<double> RTDEReceiveInterface::getTargetCurrent()
{
  return robot_state_->getTarget_current();
}

std::vector<double> RTDEReceiveInterface::getTargetMoment()
{
  return robot_state_->getTarget_moment();
}

std::vector<double> RTDEReceiveInterface::getActualQ()
{
  return robot_state_->getActual_q();
}

std::vector<double> RTDEReceiveInterface::getActualQd()
{
  return robot_state_->getActual_qd();
}

std::vector<double> RTDEReceiveInterface::getActualCurrent()
{
  return robot_state_->getActual_current();
}

std::vector<double> RTDEReceiveInterface::getJointControlOutput()
{
  return robot_state_->getJoint_control_output();
}

std::vector<double> RTDEReceiveInterface::getActualTCPPose()
{
  return robot_state_->getActual_TCP_pose();
}

std::vector<double> RTDEReceiveInterface::getActualTCPSpeed()
{
  return robot_state_->getActual_TCP_speed();
}

std::vector<double> RTDEReceiveInterface::getActualTCPForce()
{
  return robot_state_->getActual_TCP_force();
}

std::vector<double> RTDEReceiveInterface::getTargetTCPPose()
{
  return robot_state_->getTarget_TCP_pose();
}

std::vector<double> RTDEReceiveInterface::getTargetTCPSpeed()
{
  return robot_state_->getTarget_TCP_speed();
}

uint64_t RTDEReceiveInterface::getActualDigitalInputBits()
{
  return robot_state_->getActual_digital_input_bits();
}

std::vector<double> RTDEReceiveInterface::getJointTemperatures()
{
  return robot_state_->getJoint_temperatures();
}

double RTDEReceiveInterface::getActualExecutionTime()
{
  return robot_state_->getActual_execution_time();
}

int32_t RTDEReceiveInterface::getRobotMode()
{
  return robot_state_->getRobot_mode();
}

uint32_t RTDEReceiveInterface::getRobotStatus()
{
  return robot_state_->getRobot_status();
}

std::vector<int32_t> RTDEReceiveInterface::getJointMode()
{
  return robot_state_->getJoint_mode();
}

int32_t RTDEReceiveInterface::getSafetyMode()
{
  return robot_state_->getSafety_mode();
}

std::vector<double> RTDEReceiveInterface::getActualToolAccelerometer()
{
  return robot_state_->getActual_tool_accelerometer();
}

double RTDEReceiveInterface::getSpeedScaling()
{
  return robot_state_->getSpeed_scaling();
}

double RTDEReceiveInterface::getTargetSpeedFraction()
{
  return robot_state_->getTarget_speed_fraction();
}

double RTDEReceiveInterface::getActualMomentum()
{
  return robot_state_->getActual_momentum();
}

double RTDEReceiveInterface::getActualMainVoltage()
{
  return robot_state_->getActual_main_voltage();
}

double RTDEReceiveInterface::getActualRobotVoltage()
{
  return robot_state_->getActual_robot_voltage();
}

double RTDEReceiveInterface::getActualRobotCurrent()
{
  return robot_state_->getActual_robot_current();
}

std::vector<double> RTDEReceiveInterface::getActualJointVoltage()
{
  return robot_state_->getActual_joint_voltage();
}

uint64_t RTDEReceiveInterface::getActualDigitalOutputBits()
{
  return robot_state_->getActual_digital_output_bits();
}

uint32_t RTDEReceiveInterface::getRuntimeState()
{
  return robot_state_->getRuntime_state();
}

double RTDEReceiveInterface::getStandardAnalogInput0()
{
  return robot_state_->getStandard_analog_input_0();
}

double RTDEReceiveInterface::getStandardAnalogInput1()
{
  return robot_state_->getStandard_analog_input_1();
}

double RTDEReceiveInterface::getStandardAnalogOutput0()
{
  return robot_state_->getStandard_analog_output_0();
}

double RTDEReceiveInterface::getStandardAnalogOutput1()
{
  return robot_state_->getStandard_analog_output_1();
}

}  // namespace ur_rtde