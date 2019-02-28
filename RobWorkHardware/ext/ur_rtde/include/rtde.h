#ifndef RTDE_LIBRARY_H
#define RTDE_LIBRARY_H

#include <rtde_export.h>
#include <robot_state.h>
#include <cstdint>
#include <string>
#include <utility>
#include <boost/asio.hpp>

class RTDE
{
 public:
  RTDE_EXPORT explicit RTDE(const std::string hostname, int port = 30004);

  RTDE_EXPORT virtual ~RTDE();

  class RobotCommand
  {
   public:
    enum Type
    {
      NO_CMD = 0,
      MOVEJ = 1,
      MOVEJ_IK = 2,
      MOVEL = 3,
      MOVEL_FK = 4,
      MOVEC = 5,
      FORCE_MODE_START = 6,
      FORCE_MODE_UPDATE = 7,
      FORCE_MODE_STOP = 8,
      ZERO_FT_SENSOR = 9,
      SPEEDJ = 10,
      SPEEDL = 11,
      SERVOJ = 12,
      SERVOC = 13,
      SET_STD_DIGITAL_OUT = 14,
      SET_TOOL_DIGITAL_OUT = 15,
      SERVO_UPDATE = 16,
      SERVO_STOP = 17,
      STOP = 255
    };

    enum Recipe
    {
      RECIPE_1 = 1,
      RECIPE_2 = 2,
      RECIPE_3 = 3,
      RECIPE_4 = 4,
      RECIPE_5 = 5,
      RECIPE_6 = 6,
      RECIPE_7 = 7,
      RECIPE_8 = 8,
      RECIPE_9 = 9
    };

    RTDE_EXPORT RobotCommand() : type_(NO_CMD), recipe_id_(1)
    {
    }

    Type type_ = NO_CMD;
    std::uint8_t recipe_id_;
    std::vector<double> val_;
    std::vector<int> selection_vector_;
    std::int32_t movec_mode_;
    std::int32_t force_mode_type_;
    std::uint8_t std_digital_out_;
    std::uint8_t std_digital_out_mask_;
    std::uint8_t std_tool_out_;
    std::uint8_t std_tool_out_mask_;
  };

  enum RTDECommand
  {
    RTDE_REQUEST_PROTOCOL_VERSION = 86,       // ascii V
    RTDE_GET_URCONTROL_VERSION = 118,         // ascii v
    RTDE_TEXT_MESSAGE = 77,                   // ascii M
    RTDE_DATA_PACKAGE = 85,                   // ascii U
    RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79,  // ascii O
    RTDE_CONTROL_PACKAGE_SETUP_INPUTS = 73,   // ascii I
    RTDE_CONTROL_PACKAGE_START = 83,          // ascii S
    RTDE_CONTROL_PACKAGE_PAUSE = 80           // ascii P
  };

  enum class ConnectionState : std::uint8_t
  {
    DISCONNECTED = 0,
    CONNECTED = 1,
    STARTED = 2,
    PAUSED = 3
  };

 public:
  RTDE_EXPORT void connect();
  RTDE_EXPORT void disconnect();
  RTDE_EXPORT bool isConnected();

  RTDE_EXPORT bool negotiateProtocolVersion();
  RTDE_EXPORT std::tuple<std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t> getControllerVersion();
  RTDE_EXPORT void receive();
  RTDE_EXPORT void receiveData(std::shared_ptr<RobotState>& robot_state);
  RTDE_EXPORT void send(const RobotCommand& robot_cmd);
  RTDE_EXPORT void sendAll(const std::uint8_t& command, std::string payload = "");
  RTDE_EXPORT void sendStart();
  RTDE_EXPORT void sendPause();
  RTDE_EXPORT bool sendOutputSetup(const std::vector<std::string>& output_names, double frequency);
  RTDE_EXPORT bool sendInputSetup(const std::vector<std::string>& input_names);

 private:
  std::string hostname_;
  int port_;
  ConnectionState conn_state_;
  std::vector<std::string> output_types_;
  std::vector<std::string> output_names_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
  std::shared_ptr<boost::asio::ip::tcp::resolver> resolver_;
};

#endif
