#ifndef RTDE_DASHBOARD_CLIENT_H
#define RTDE_DASHBOARD_CLIENT_H

#include <rtde_export.h>
#include <string>
#include <boost/asio.hpp>

namespace ur_rtde
{
class DashboardClient
{
 public:
  RTDE_EXPORT explicit DashboardClient(std::string hostname, int port = 29999);

  RTDE_EXPORT virtual ~DashboardClient();

  enum class ConnectionState : std::uint8_t
  {
    DISCONNECTED = 0,
    CONNECTED = 1,
  };

 public:
  RTDE_EXPORT void connect();
  RTDE_EXPORT bool isConnected();
  RTDE_EXPORT void disconnect();
  RTDE_EXPORT void send(const std::string &str);
  RTDE_EXPORT void loadURP(const std::string &urp_name);
  RTDE_EXPORT void play();
  RTDE_EXPORT void stop();
  RTDE_EXPORT void pause();
  RTDE_EXPORT void quit();
  RTDE_EXPORT void shutdown();
  RTDE_EXPORT bool running();
  RTDE_EXPORT void popup(const std::string &message);
  RTDE_EXPORT void closePopup();
  RTDE_EXPORT std::string programState();
  RTDE_EXPORT void powerOn();
  RTDE_EXPORT void powerOff();
  RTDE_EXPORT void brakeRelease();
  RTDE_EXPORT void unlockProtectiveStop();

 private:
  std::string hostname_;
  int port_;
  ConnectionState conn_state_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
  std::shared_ptr<boost::asio::ip::tcp::resolver> resolver_;
};

}  // namespace ur_rtde

#endif  // RTDE_DASHBOARD_CLIENT_H
