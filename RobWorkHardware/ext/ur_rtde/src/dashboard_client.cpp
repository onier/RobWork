#include <dashboard_client.h>
#include <iostream>

using boost::asio::ip::tcp;

DashboardClient::DashboardClient(std::string hostname, int port)
    : hostname_(std::move(hostname)), port_(port), conn_state_(ConnectionState::DISCONNECTED)
{
}

DashboardClient::~DashboardClient() = default;

void DashboardClient::connect()
{
  io_service_ = std::make_shared<boost::asio::io_service>();
  socket_ = std::make_shared<tcp::socket>(*io_service_);
  socket_->open(boost::asio::ip::tcp::v4());
  boost::asio::ip::tcp::no_delay no_delay_option(true);
  boost::asio::socket_base::reuse_address sol_reuse_option(true);
  socket_->set_option(no_delay_option);
  socket_->set_option(sol_reuse_option);
  resolver_ = std::make_shared<tcp::resolver>(*io_service_);
  tcp::resolver::query query(hostname_, std::to_string(port_));
  boost::asio::connect(*socket_, resolver_->resolve(query));
  conn_state_ = ConnectionState::CONNECTED;
  std::cout << "Connected successfully to UR dashboard server: " << hostname_ << " at " << port_ << std::endl;
}

bool DashboardClient::isConnected()
{
  return conn_state_ == ConnectionState::CONNECTED;
}

void DashboardClient::disconnect()
{
  // Close socket
  socket_->close();
  conn_state_ = ConnectionState::DISCONNECTED;
  std::cout << "Dashboard Client - Socket disconnected" << std::endl;
}

void DashboardClient::send(const std::string &str)
{
  boost::asio::write(*socket_, boost::asio::buffer(str));
}

void DashboardClient::loadURP(const std::string &urp_name)
{
  std::string load_urp = "load "+urp_name+"\n";
  send(load_urp);
}

void DashboardClient::play()
{
  std::string play = "play\n";
  send(play);
}

void DashboardClient::stop()
{
  std::string stop = "stop\n";
  send(stop);
}

void DashboardClient::pause()
{
  std::string pause = "pause\n";
  send(pause);
}

void DashboardClient::quit()
{
  std::string quit = "quit\n";
  send(quit);
}
void DashboardClient::shutdown()
{
  std::string shutdown = "shutdown\n";
  send(shutdown);
}

bool DashboardClient::running()
{
  return false;
}

void DashboardClient::popup(const std::string &message)
{
  std::string popup = "popup "+message+"\n";
  send(popup);
}

void DashboardClient::closePopup()
{
  std::string close_popup = "close popup\n";
  send(close_popup);
}

std::string DashboardClient::programState()
{
  return std::string();
}

void DashboardClient::powerOn()
{
  std::string power_on = "power on\n";
  send(power_on);
}

void DashboardClient::powerOff()
{
  std::string power_off = "power off\n";
  send(power_off);
}

void DashboardClient::brakeRelease()
{
  std::string brake_release = "brake release\n";
  send(brake_release);
}

void DashboardClient::unlockProtectiveStop()
{
  std::string unlock_p_stop = "unlock protective stop\n";
  send(unlock_p_stop);
}
