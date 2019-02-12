#include <script_client.h>
#include <iostream>
#include <string>
#include <fstream>
#include <streambuf>
#include "rtde_control_script.h"

using boost::asio::ip::tcp;

ScriptClient::ScriptClient(std::string hostname, int port)
    : hostname_(std::move(hostname)), port_(port), conn_state_(ConnectionState::DISCONNECTED)
{
}

ScriptClient::~ScriptClient() = default;

void ScriptClient::connect()
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
  std::cout << "Connected successfully to UR script server: " << hostname_ << " at " << port_ << std::endl;
}

bool ScriptClient::isConnected()
{
  return conn_state_ == ConnectionState::CONNECTED;
}

void ScriptClient::disconnect()
{
  // Close socket
  socket_->close();
  conn_state_ = ConnectionState::DISCONNECTED;
  std::cout << "Script Client - Socket disconnected" << std::endl;
}

bool ScriptClient::sendScriptCommand(const std::string &cmd_str)
{
  if(isConnected() && !cmd_str.empty())
  {
    boost::asio::write(*socket_, boost::asio::buffer(cmd_str));
  }
  else
  {
    std::cerr << "Please connect to the controller before calling sendScriptCommand()" << std::endl;
    return false;
  }

  return true;
}

bool ScriptClient::sendScript()
{
  if(isConnected() && !UR_SCRIPT.empty())
  {
    boost::asio::write(*socket_, boost::asio::buffer(UR_SCRIPT));
  }
  else
  {
    std::cerr << "Please connect to the controller before calling sendScript()" << std::endl;
    return false;
  }

  return true;
}

bool ScriptClient::sendScript(const std::string &file_name)
{
  // Read in the UR script file
  // Notice! We use this method as it allocates the memory up front, strictly for performance.
  std::string str;
  std::ifstream file(file_name.c_str());
  if (file)
  {
    file.seekg(0, std::ios::end);
    str.reserve(file.tellg());
    file.seekg(0, std::ios::beg);
    // Do not remove the redundant parentheses, this is to avoid the most vexing parse!
    str.assign((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  }
  else
  {
    std::cerr << "There was an error reading the provided script file: " << file_name << std::endl;
    return false;
  }

  if(isConnected() && !str.empty())
  {
    boost::asio::write(*socket_, boost::asio::buffer(str));
  }
  else
  {
    std::cerr << "Please connect to the controller before calling sendScript()" << std::endl;
    return false;
  }

  return true;
}