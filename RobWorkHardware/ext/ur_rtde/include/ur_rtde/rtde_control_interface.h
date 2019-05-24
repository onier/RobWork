#ifndef RTDE_RTDE_CONTROL_INTERFACE_H
#define RTDE_RTDE_CONTROL_INTERFACE_H

#include <rtde_export.h>
#include <ur_rtde/rtde.h>
#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/script_client.h>
#include <thread>
#include <sstream>

#define MAJOR_VERSION 0
#define CB3_MAJOR_VERSION 3
#define UR_CONTROLLER_RDY_FOR_CMD 1
#define UR_CONTROLLER_CMD_RECEIVED 0
#define UR_CONTROLLER_DONE_WITH_CMD 1
#define UR_EXECUTION_TIMEOUT 30
#define UR_PATH_EXECUTION_TIMEOUT 120
#define UR_GET_READY_TIMEOUT 3
#define UR_CMD_RECEIVE_TIMEOUT 3

#define UR_VELOCITY_MAX 1.0
#define UR_VELOCITY_ABSOLUTE_MAX 3.14
#define UR_VELOCITY_MIN 0
#define UR_ACCELERATION_MAX 2.0
#define UR_ACCELERATION_ABSOLUTE_MAX 5.0
#define UR_ACCELERATION_MIN 0
#define UR_SERVO_LOOKAHEAD_TIME_MAX 0.2
#define UR_SERVO_LOOKAHEAD_TIME_MIN 0.03
#define UR_SERVO_GAIN_MAX 2000
#define UR_SERVO_GAIN_MIN 100
#define UR_BLEND_MAX 2.0
#define UR_BLEND_MIN 0.0

namespace ur_rtde
{
class RTDEControlInterface
{
 public:
  RTDE_EXPORT explicit RTDEControlInterface(std::string hostname, int port = 30004);

  RTDE_EXPORT virtual ~RTDEControlInterface();

  enum RobotStatus
  {
    ROBOT_STATUS_POWER_ON = 0,
    ROBOT_STATUS_PROGRAM_RUNNING = 1,
    ROBOT_STATUS_TEACH_BUTTON_PRESSED = 2,
    ROBOT_STATUS_POWER_BUTTON_PRESSED = 3
  };

  /**
   * @brief In the event of an error, this function can be used to resume operation by reuploading the RTDE control
   * script. This will only happen if a script is not already running on the controller.
   */
  RTDE_EXPORT bool reuploadScript();

  /**
    * @brief Send a custom ur script to the controller
    * @param script the custom ur script to be sent to the controller specified as a string
    */
  RTDE_EXPORT bool sendCustomScript(const std::string &script);

  /**
    * @brief Send a custom ur script file to the controller
    * @param file_path the file path to the custom ur script file
    */
  RTDE_EXPORT bool sendCustomScriptFile(const std::string &file_path);

  /**
    * @brief This function will stop whatever the robot is doing, and terminate script on controller
    */
  RTDE_EXPORT void stopRobot();

  /**
    * @brief Move to joint position (linear in joint-space)
    * @param q joint positions
    * @param speed joint speed of leading axis [rad/s]
    * @param acceleration joint acceleration of leading axis [rad/s^2]
    */
  RTDE_EXPORT bool moveJ(const std::vector<double> &q, double speed, double acceleration);

  /**
    * @brief Move to each joint position specified in a path
    * @param path with joint positions that includes acceleration, speed and blend for each position
    */
  RTDE_EXPORT bool moveJ(const std::vector<std::vector<double>> &path);

  /**
    * @brief Move to pose (linear in joint-space)
    * @param pose target pose
    * @param speed joint speed of leading axis [rad/s]
    * @param acceleration joint acceleration of leading axis [rad/s^2]
    */
  RTDE_EXPORT bool moveJ_IK(const std::vector<double> &pose, double speed, double acceleration);

  /**
    * @brief Move to position (linear in tool-space)
    * @param pose target pose
    * @param speed tool speed [m/s]
    * @param acceleration tool acceleration [m/s^2]
    */
  RTDE_EXPORT bool moveL(const std::vector<double> &pose, double speed, double acceleration);

  /**
    * @brief Move to each pose specified in a path
    * @param path with tool poses that includes acceleration, speed and blend for each position
    */
  RTDE_EXPORT bool moveL(const std::vector<std::vector<double>> &path);

  /**
    * @brief Move to position (linear in tool-space)
    * @param q joint positions
    * @param speed tool speed [m/s]
    * @param acceleration tool acceleration [m/s^2]
    */
  RTDE_EXPORT bool moveL_FK(const std::vector<double> &q, double speed, double acceleration);

  /**
    * @brief Move Circular: Move to position (circular in tool-space)
    * @param pose_via path point (note: only position is used)
    * @param pose_to target pose (note: only position is used in Fixed orientation mode).
    * @param speed tool speed [m/s]
    * @param acceleration tool acceleration [m/s^2]
    * @param mode 0: Unconstrained mode. Interpolate orientation from current pose to target pose (pose_to)
    * 1: Fixed mode. Keep orientation constant relative to the tangent of the circular arc (starting from current pose)
    */
  RTDE_EXPORT bool moveC(const std::vector<double> &pose_via, const std::vector<double> &pose_to, double speed,
                         double acceleration, int mode);

  /**
    * @brief Joint speed - Accelerate linearly in joint space and continue with constant joint speed
    * @param qd joint speeds [rad/s]
    * @param acceleration joint acceleration [rad/s^2] (of leading axis)
    * @param time time [s] before the function returns (optional)
    */
  RTDE_EXPORT bool speedJ(const std::vector<double> &qd, double acceleration, double time = 0.0);

  /**
    * @brief Tool speed - Accelerate linearly in Cartesian space and continue with constant tool speed. The time t is
    * optional;
    * @param xd tool speed [m/s] (spatial vector)
    * @param acceleration tool position acceleration [m/s^2]
    * @param time time [s] before the function returns (optional)
    */
  RTDE_EXPORT bool speedL(const std::vector<double> &xd, double acceleration, double time = 0.0);

  /**
    * @brief Servo to position (linear in joint-space)
    * @param q joint positions [rad]
    * @param speed NOT used in current version
    * @param acceleration NOT used in current version
    * @param time time where the command is controlling the robot. The function is blocking for time t [S]
    * @param lookahead_time time [S], range [0.03,0.2] smoothens the trajectory with this lookahead time
    * @param gain proportional gain for following target position, range [100,2000]
    */
  RTDE_EXPORT bool servoJ(const std::vector<double> &q, double speed, double acceleration, double time,
                          double lookahead_time, double gain);

  /**
    * @brief Stop servos
    */
  RTDE_EXPORT bool servoStop();

  /**
    * @brief Stop speeding
    */
  RTDE_EXPORT bool speedStop();

  /**
    * @brief Servo to position (circular in tool-space). Accelerates to and moves with constant tool speed v.
    * @param pose target pose
    * @param speed tool speed [m/s]
    * @param acceleration tool acceleration [m/s^2]
    * @param blend blend radius (of target pose) [m]
    */
  RTDE_EXPORT bool servoC(const std::vector<double> &pose, double speed, double acceleration, double blend);

  /**
    * @brief Set robot to be controlled in force mode
    * @param task_frame A pose vector that defines the force frame relative to the base frame.
    * @param selection_vector A 6d vector of 0s and 1s. 1 means that the robot will be compliant in the corresponding
    * axis of the task frame
    * @param wrench The forces/torques the robot will apply to its environment. The robot adjusts its position
    * along/about compliant axis in order to achieve the specified force/torque. Values have no effect for
    * non-compliant axes
    * @param type An integer [1;3] specifying how the robot interprets the force frame.
    * 1: The force frame is transformed in a way such that its y-axis is aligned with a vector pointing from the
    * robot tcp towards the origin of the force frame. 2: The force frame is not transformed. 3: The force frame is
    * transformed in a way such that its x-axis is the projection of the robot tcp velocity vector onto the x-y plane
    * of the force frame.
    * @param limits (Float) 6d vector. For compliant axes, these values are the maximum allowed tcp speed along/about
    * the axis. For non-compliant axes, these values are the maximum allowed deviation along/about an axis between the
    * actual tcp position and the one set by the program.
    */
  RTDE_EXPORT bool forceModeStart(const std::vector<double> &task_frame, const std::vector<int> &selection_vector,
                                  const std::vector<double> &wrench, int type, const std::vector<double> &limits);

  /**
    * @brief Update the wrench the robot will apply to its environment
    * @param wrench The forces/torques the robot will apply to its environment. The robot adjusts its position
    * along/about compliant axis in order to achieve the specified force/torque. Values have no effect for
    * non-compliant axes
    */
  RTDE_EXPORT bool forceModeUpdate(const std::vector<double> &wrench);

  /**
    * @brief Resets the robot mode from force mode to normal operation.
    */
  RTDE_EXPORT bool forceModeStop();

  /**
    * @brief Zeroes the TCP force/torque measurement from the builtin force/torque sensor by subtracting the current
    * measurement from the subsequent.
    */
  RTDE_EXPORT bool zeroFtSensor();

  /**
    * @brief Set standard digital output signal level
    * @param output_id The number (id) of the output, integer: [0:7]
    * @param signal_level The signal level. (boolean)
    */
  RTDE_EXPORT bool setStandardDigitalOut(std::uint8_t output_id, bool signal_level);

  /**
    * @brief Set tool digital output signal level
    * @param output_id The number (id) of the output, integer: [0:1]
    * @param signal_level The signal level. (boolean)
    */
  RTDE_EXPORT bool setToolDigitalOut(std::uint8_t output_id, bool signal_level);

  /**
    * @brief Set payload
    * @param mass Mass in kilograms
    * @param cog Center of Gravity, a vector [CoGx, CoGy, CoGz] specifying the displacement (in meters) from the
    * toolmount. If not specified the current CoG will be used.
    */
  RTDE_EXPORT bool setPayload(double mass, const std::vector<double> &cog = {});

  /**
    * @brief Set robot in freedrive mode. In this mode the robot can be moved around by hand in the same way as
    * by pressing the "freedrive" button. The robot will not be able to follow a trajectory (eg. a movej) in this mode.
    */
  RTDE_EXPORT bool teachMode();

  /**
    * @brief Set robot back in normal position control mode after freedrive mode.
    */
  RTDE_EXPORT bool endTeachMode();

  /**
    * @brief Sets the damping parameter in force mode.
    * @param damping Between 0 and 1, default value is 0.005
    *
    * A value of 1 is full damping, so the robot will decellerate quickly if no force is present.
    * A value of 0 is no damping, here the robot will maintain the speed.
    *
    * The value is stored until this function is called again. Call this function
    * before force mode is entered (otherwise default value will be used).
    */
  RTDE_EXPORT bool forceModeSetDamping(double damping);

  /**
    * @brief Scales the gain in force mode.
    * @param scaling scaling parameter between 0 and 2, default is 1.
    *
    * A value larger than 1 can make force mode unstable, e.g. in case of collisions or pushing against hard surfaces.
    *
    * The value is stored until this function is called again. Call this function before force mode is entered
    * (otherwise default value will be used)
    */
  RTDE_EXPORT bool forceModeSetGainScaling(double scaling);

  /**
    * @brief Set the speed slider on the controller
    * @param speed set the speed slider on the controller as a fraction value between 0 and 1 (1 is 100%)
    */
  RTDE_EXPORT bool setSpeedSlider(double speed);

  /**
    * @brief Set Analog output voltage
    * @param output_id The number (id) of the output, integer: [0:1]
    * @param voltage_ratio voltage set as a (ratio) of the voltage span [0..1], 1 means full voltage.
    */
  RTDE_EXPORT bool setAnalogOutputVoltage(std::uint8_t output_id, double voltage_ratio);

  /**
    * @brief Set Analog output current
    * @param output_id The number (id) of the output, integer: [0:1]
    * @param current_ratio current set as a (ratio) of the current span [0..1], 1 means full current.
    */
  RTDE_EXPORT bool setAnalogOutputCurrent(std::uint8_t output_id, double current_ratio);

  /**
    * @brief Returns true if a program is running on the controller, otherwise it returns false
    */
  RTDE_EXPORT bool isProgramRunning();

 private:
  bool sendCommand(const RTDE::RobotCommand &cmd);

  void sendClearCommand();

  int getControlScriptState();

  void verifyValueIsWithin(const double &value, const double &min, const double &max);

  std::string prepareCmdScript(const std::vector<std::vector<double>> &path, const std::string &cmd);

 private:
  std::string hostname_;
  int port_;
  std::shared_ptr<RTDE> rtde_;
  std::shared_ptr<DashboardClient> db_client_;
  std::shared_ptr<ScriptClient> script_client_;
  std::shared_ptr<RobotState> robot_state_;
};

}  // namespace ur_rtde

#endif  // RTDE_RTDE_CONTROL_INTERFACE_H
