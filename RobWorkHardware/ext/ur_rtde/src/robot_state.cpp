#include <ur_rtde/robot_state.h>

namespace ur_rtde
{
RobotState::RobotState()
{
}

RobotState::~RobotState()
{
}

double RobotState::getTimestamp()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return timestamp_;
}
void RobotState::setTimestamp(double timestamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::timestamp_ = timestamp;
}
const std::vector<double> &RobotState::getTarget_q()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return target_q_;
}
void RobotState::setTarget_q(const std::vector<double> &target_q)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::target_q_ = target_q;
}
const std::vector<double> &RobotState::getTarget_qd()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return target_qd_;
}
void RobotState::setTarget_qd(const std::vector<double> &target_qd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::target_qd_ = target_qd;
}
const std::vector<double> &RobotState::getTarget_qdd()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return target_qdd_;
}
void RobotState::setTarget_qdd(const std::vector<double> &target_qdd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::target_qdd_ = target_qdd;
}
const std::vector<double> &RobotState::getTarget_current()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return target_current_;
}
void RobotState::setTarget_current(const std::vector<double> &target_current)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::target_current_ = target_current;
}
const std::vector<double> &RobotState::getTarget_moment()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return target_moment_;
}
void RobotState::setTarget_moment(const std::vector<double> &target_moment)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::target_moment_ = target_moment;
}
const std::vector<double> &RobotState::getActual_q()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_q_;
}
void RobotState::setActual_q(const std::vector<double> &actual_q)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::actual_q_ = actual_q;
}
const std::vector<double> &RobotState::getActual_qd()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_qd_;
}
void RobotState::setActual_qd(const std::vector<double> &actual_qd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::actual_qd_ = actual_qd;
}
const std::vector<double> &RobotState::getActual_current()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_current_;
}
void RobotState::setActual_current(const std::vector<double> &actual_current)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::actual_current_ = actual_current;
}
const std::vector<double> &RobotState::getJoint_control_output()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_control_output_;
}
void RobotState::setJoint_control_output(const std::vector<double> &joint_control_output)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::joint_control_output_ = joint_control_output;
}
const std::vector<double> &RobotState::getActual_TCP_pose()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_TCP_pose_;
}
void RobotState::setActual_TCP_pose(const std::vector<double> &actual_TCP_pose)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::actual_TCP_pose_ = actual_TCP_pose;
}
const std::vector<double> &RobotState::getActual_TCP_speed()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_TCP_speed_;
}
void RobotState::setActual_TCP_speed(const std::vector<double> &actual_TCP_speed)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::actual_TCP_speed_ = actual_TCP_speed;
}
const std::vector<double> &RobotState::getActual_TCP_force()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_TCP_force_;
}
void RobotState::setActual_TCP_force(const std::vector<double> &actual_TCP_force)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::actual_TCP_force_ = actual_TCP_force;
}
const std::vector<double> &RobotState::getTarget_TCP_pose()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return target_TCP_pose_;
}
void RobotState::setTarget_TCP_pose(const std::vector<double> &target_TCP_pose)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::target_TCP_pose_ = target_TCP_pose;
}
const std::vector<double> &RobotState::getTarget_TCP_speed()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return target_TCP_speed_;
}
void RobotState::setTarget_TCP_speed(const std::vector<double> &target_TCP_speed)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::target_TCP_speed_ = target_TCP_speed;
}
uint64_t RobotState::getActual_digital_input_bits()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_digital_input_bits_;
}
void RobotState::setActual_digital_input_bits(uint64_t actual_digital_input_bits)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::actual_digital_input_bits_ = actual_digital_input_bits;
}
const std::vector<double> &RobotState::getJoint_temperatures()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_temperatures_;
}
void RobotState::setJoint_temperatures(const std::vector<double> &joint_temperatures)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::joint_temperatures_ = joint_temperatures;
}
double RobotState::getActual_execution_time()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_execution_time_;
}
void RobotState::setActual_execution_time(double actual_execution_time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::actual_execution_time_ = actual_execution_time;
}
int32_t RobotState::getRobot_mode()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return robot_mode_;
}
void RobotState::setRobot_mode(int32_t robot_mode)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::robot_mode_ = robot_mode;
}
uint32_t RobotState::getRobot_status()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return robot_status_;
}
void RobotState::setRobot_status(uint32_t robot_status)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::robot_status_ = robot_status;
}
const std::vector<int32_t> &RobotState::getJoint_mode()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_mode_;
}
void RobotState::setJoint_mode(const std::vector<int32_t> &joint_mode)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::joint_mode_ = joint_mode;
}
int32_t RobotState::getSafety_mode()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return safety_mode_;
}
void RobotState::setSafety_mode(int32_t safety_mode)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::safety_mode_ = safety_mode;
}
const std::vector<double> &RobotState::getActual_tool_accelerometer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_tool_accelerometer_;
}
void RobotState::setActual_tool_accelerometer(const std::vector<double> &actual_tool_accelerometer)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::actual_tool_accelerometer_ = actual_tool_accelerometer;
}
double RobotState::getSpeed_scaling()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return speed_scaling_;
}
void RobotState::setSpeed_scaling(double speed_scaling)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::speed_scaling_ = speed_scaling;
}
double RobotState::getTarget_speed_fraction()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return target_speed_fraction_;
}
void RobotState::setTarget_speed_fraction(double target_speed_fraction)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::target_speed_fraction_ = target_speed_fraction;
}
double RobotState::getActual_momentum()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_momentum_;
}
void RobotState::setActual_momentum(double actual_momentum)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::actual_momentum_ = actual_momentum;
}
double RobotState::getActual_main_voltage()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_main_voltage_;
}
void RobotState::setActual_main_voltage(double actual_main_voltage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::actual_main_voltage_ = actual_main_voltage;
}
double RobotState::getActual_robot_voltage()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_robot_voltage_;
}
void RobotState::setActual_robot_voltage(double actual_robot_voltage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::actual_robot_voltage_ = actual_robot_voltage;
}
double RobotState::getActual_robot_current()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_robot_current_;
}
void RobotState::setActual_robot_current(double actual_robot_current)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::actual_robot_current_ = actual_robot_current;
}
const std::vector<double> &RobotState::getActual_joint_voltage()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_joint_voltage_;
}
void RobotState::setActual_joint_voltage(const std::vector<double> &actual_joint_voltage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::actual_joint_voltage_ = actual_joint_voltage;
}
uint64_t RobotState::getActual_digital_output_bits()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_digital_output_bits_;
}
void RobotState::setActual_digital_output_bits(uint64_t actual_digital_output_bits)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::actual_digital_output_bits_ = actual_digital_output_bits;
}
uint32_t RobotState::getRuntime_state()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return runtime_state_;
}
double RobotState::getStandard_analog_input_0()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return standard_analog_input_0_;
}
void RobotState::setStandard_analog_input_0(double standard_analog_input_0)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::standard_analog_input_0_ = standard_analog_input_0;
}
double RobotState::getStandard_analog_input_1()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return standard_analog_input_1_;
}
void RobotState::setStandard_analog_input_1(double standard_analog_input_1)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::standard_analog_input_1_ = standard_analog_input_1;
}
double RobotState::getStandard_analog_output_0()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return standard_analog_output_0_;
}
void RobotState::setStandard_analog_output_0(double standard_analog_output_0)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::standard_analog_output_0_ = standard_analog_output_0;
}
double RobotState::getStandard_analog_output_1()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return standard_analog_output_1_;
}
void RobotState::setStandard_analog_output_1(double standard_analog_output_1)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::standard_analog_output_1_ = standard_analog_output_1;
}
void RobotState::setRuntime_state(uint32_t runtime_state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::runtime_state_ = runtime_state;
}
uint32_t RobotState::getOutput_bit_registers0_to_31()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_bit_registers0_to_31_;
}
void RobotState::setOutput_bit_registers0_to_31(uint32_t output_bit_registers0_to_31)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_bit_registers0_to_31_ = output_bit_registers0_to_31;
}
uint32_t RobotState::getOutput_bit_registers32_to_63()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_bit_registers32_to_63_;
}
void RobotState::setOutput_bit_registers32_to_63(uint32_t output_bit_registers32_to_63)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_bit_registers32_to_63_ = output_bit_registers32_to_63;
}
int32_t RobotState::getOutput_int_register_0()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_0_;
}
void RobotState::setOutput_int_register_0(int32_t output_int_register_0)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_0_ = output_int_register_0;
}
int32_t RobotState::getOutput_int_register_1()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_1_;
}
void RobotState::setOutput_int_register_1(int32_t output_int_register_1)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_1_ = output_int_register_1;
}
int32_t RobotState::getOutput_int_register_2()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_2_;
}
void RobotState::setOutput_int_register_2(int32_t output_int_register_2)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_2_ = output_int_register_2;
}
int32_t RobotState::getOutput_int_register_3()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_3_;
}
void RobotState::setOutput_int_register_3(int32_t output_int_register_3)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_3_ = output_int_register_3;
}
int32_t RobotState::getOutput_int_register_4()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_4_;
}
void RobotState::setOutput_int_register_4(int32_t output_int_register_4)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_4_ = output_int_register_4;
}
int32_t RobotState::getOutput_int_register_5()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_5_;
}
void RobotState::setOutput_int_register_5(int32_t output_int_register_5)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_5_ = output_int_register_5;
}
int32_t RobotState::getOutput_int_register_6()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_6_;
}
void RobotState::setOutput_int_register_6(int32_t output_int_register_6)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_6_ = output_int_register_6;
}
int32_t RobotState::getOutput_int_register_7()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_7_;
}
void RobotState::setOutput_int_register_7(int32_t output_int_register_7)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_7_ = output_int_register_7;
}
int32_t RobotState::getOutput_int_register_8()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_8_;
}
void RobotState::setOutput_int_register_8(int32_t output_int_register_8)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_8_ = output_int_register_8;
}
int32_t RobotState::getOutput_int_register_9()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_9_;
}
void RobotState::setOutput_int_register_9(int32_t output_int_register_9)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_9_ = output_int_register_9;
}
int32_t RobotState::getOutput_int_register_10()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_10_;
}
void RobotState::setOutput_int_register_10(int32_t output_int_register_10)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_10_ = output_int_register_10;
}
int32_t RobotState::getOutput_int_register_11()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_11_;
}
void RobotState::setOutput_int_register_11(int32_t output_int_register_11)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_11_ = output_int_register_11;
}
int32_t RobotState::getOutput_int_register_12()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_12_;
}
void RobotState::setOutput_int_register_12(int32_t output_int_register_12)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_12_ = output_int_register_12;
}
int32_t RobotState::getOutput_int_register_13()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_13_;
}
void RobotState::setOutput_int_register_13(int32_t output_int_register_13)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_13_ = output_int_register_13;
}
int32_t RobotState::getOutput_int_register_14()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_14_;
}
void RobotState::setOutput_int_register_14(int32_t output_int_register_14)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_14_ = output_int_register_14;
}
int32_t RobotState::getOutput_int_register_15()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_15_;
}
void RobotState::setOutput_int_register_15(int32_t output_int_register_15)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_15_ = output_int_register_15;
}
int32_t RobotState::getOutput_int_register_16()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_16_;
}
void RobotState::setOutput_int_register_16(int32_t output_int_register_16)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_16_ = output_int_register_16;
}
int32_t RobotState::getOutput_int_register_17()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_17_;
}
void RobotState::setOutput_int_register_17(int32_t output_int_register_17)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_17_ = output_int_register_17;
}
int32_t RobotState::getOutput_int_register_18()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_18_;
}
void RobotState::setOutput_int_register_18(int32_t output_int_register_18)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_18_ = output_int_register_18;
}
int32_t RobotState::getOutput_int_register_19()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_19_;
}
void RobotState::setOutput_int_register_19(int32_t output_int_register_19)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_19_ = output_int_register_19;
}
int32_t RobotState::getOutput_int_register_20()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_20_;
}
void RobotState::setOutput_int_register_20(int32_t output_int_register_20)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_20_ = output_int_register_20;
}
int32_t RobotState::getOutput_int_register_21()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_21_;
}
void RobotState::setOutput_int_register_21(int32_t output_int_register_21)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_21_ = output_int_register_21;
}
int32_t RobotState::getOutput_int_register_22()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_22_;
}
void RobotState::setOutput_int_register_22(int32_t output_int_register_22)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_22_ = output_int_register_22;
}
int32_t RobotState::getOutput_int_register_23()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_int_register_23_;
}
void RobotState::setOutput_int_register_23(int32_t output_int_register_23)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_int_register_23_ = output_int_register_23;
}
double RobotState::getOutput_double_register_0()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_0_;
}
void RobotState::setOutput_double_register_0(double output_double_register_0)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_0_ = output_double_register_0;
}
double RobotState::getOutput_double_register_1()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_1_;
}
void RobotState::setOutput_double_register_1(double output_double_register_1)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_1_ = output_double_register_1;
}
double RobotState::getOutput_double_register_2()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_2_;
}
void RobotState::setOutput_double_register_2(double output_double_register_2)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_2_ = output_double_register_2;
}
double RobotState::getOutput_double_register_3()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_3_;
}
void RobotState::setOutput_double_register_3(double output_double_register_3)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_3_ = output_double_register_3;
}
double RobotState::getOutput_double_register_4()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_4_;
}
void RobotState::setOutput_double_register_4(double output_double_register_4)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_4_ = output_double_register_4;
}
double RobotState::getOutput_double_register_5()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_5_;
}
void RobotState::setOutput_double_register_5(double output_double_register_5)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_5_ = output_double_register_5;
}
double RobotState::getOutput_double_register_6()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_6_;
}
void RobotState::setOutput_double_register_6(double output_double_register_6)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_6_ = output_double_register_6;
}
double RobotState::getOutput_double_register_7()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_7_;
}
void RobotState::setOutput_double_register_7(double output_double_register_7)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_7_ = output_double_register_7;
}
double RobotState::getOutput_double_register_8()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_8_;
}
void RobotState::setOutput_double_register_8(double output_double_register_8)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_8_ = output_double_register_8;
}
double RobotState::getOutput_double_register_9()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_9_;
}
void RobotState::setOutput_double_register_9(double output_double_register_9)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_9_ = output_double_register_9;
}
double RobotState::getOutput_double_register_10()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_10_;
}
void RobotState::setOutput_double_register_10(double output_double_register_10)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_10_ = output_double_register_10;
}
double RobotState::getOutput_double_register_11()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_11_;
}
void RobotState::setOutput_double_register_11(double output_double_register_11)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_11_ = output_double_register_11;
}
double RobotState::getOutput_double_register_12()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_12_;
}
void RobotState::setOutput_double_register_12(double output_double_register_12)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_12_ = output_double_register_12;
}
double RobotState::getOutput_double_register_13()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_13_;
}
void RobotState::setOutput_double_register_13(double output_double_register_13)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_13_ = output_double_register_13;
}
double RobotState::getOutput_double_register_14()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_14_;
}
void RobotState::setOutput_double_register_14(double output_double_register_14)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_14_ = output_double_register_14;
}
double RobotState::getOutput_double_register_15()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_15_;
}
void RobotState::setOutput_double_register_15(double output_double_register_15)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_15_ = output_double_register_15;
}
double RobotState::getOutput_double_register_16()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_16_;
}
void RobotState::setOutput_double_register_16(double output_double_register_16)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_16_ = output_double_register_16;
}
double RobotState::getOutput_double_register_17()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_17_;
}
void RobotState::setOutput_double_register_17(double output_double_register_17)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_17_ = output_double_register_17;
}
double RobotState::getOutput_double_register_18()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_18_;
}
void RobotState::setOutput_double_register_18(double output_double_register_18)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_18_ = output_double_register_18;
}
double RobotState::getOutput_double_register_19()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_19_;
}
void RobotState::setOutput_double_register_19(double output_double_register_19)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_19_ = output_double_register_19;
}
double RobotState::getOutput_double_register_20()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_20_;
}
void RobotState::setOutput_double_register_20(double output_double_register_20)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_20_ = output_double_register_20;
}
double RobotState::getOutput_double_register_21()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_21_;
}
void RobotState::setOutput_double_register_21(double output_double_register_21)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_21_ = output_double_register_21;
}
double RobotState::getOutput_double_register_22()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_22_;
}
void RobotState::setOutput_double_register_22(double output_double_register_22)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_22_ = output_double_register_22;
}
double RobotState::getOutput_double_register_23()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return output_double_register_23_;
}
void RobotState::setOutput_double_register_23(double output_double_register_23)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotState::output_double_register_23_ = output_double_register_23;
}

}  // namespace ur_rtde