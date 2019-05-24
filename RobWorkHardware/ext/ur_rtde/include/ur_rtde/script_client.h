#ifndef RTDE_SCRIPT_CLIENT_H
#define RTDE_SCRIPT_CLIENT_H

#include <rtde_export.h>
#include <string>
#include <boost/asio.hpp>
#include <boost/filesystem.hpp>

namespace ur_rtde
{
class ScriptClient
{
 public:
  RTDE_EXPORT explicit ScriptClient(std::string hostname, uint32_t major_control_version, int port = 30002);

  RTDE_EXPORT virtual ~ScriptClient();

  enum class ConnectionState : std::uint8_t
  {
    DISCONNECTED = 0,
    CONNECTED = 1,
  };

 public:
  RTDE_EXPORT void connect();
  RTDE_EXPORT void disconnect();
  RTDE_EXPORT bool isConnected();
  RTDE_EXPORT bool sendScript();
  RTDE_EXPORT bool sendScript(const std::string &file_name);
  RTDE_EXPORT bool sendScriptCommand(const std::string &cmd_str);

 private:
  std::string hostname_;
  uint32_t major_control_version_;
  int port_;
  ConnectionState conn_state_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
  std::shared_ptr<boost::asio::ip::tcp::resolver> resolver_;
};

}  // namespace ur_rtde

#endif  // RTDE_SCRIPT_CLIENT_H
