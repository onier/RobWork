#include <rtde_receive_interface.h>
#include <iostream>
#include <chrono>
#include <numeric>

using namespace std::chrono;
int main(int argc, char *argv[])
{
  RTDEReceiveInterface rtde_receive("127.0.0.1");

  while(1)
  {
    std::cout << "Actual q is: " << std::endl;
    for (const auto &d : rtde_receive.getActualQ())
      std::cout << d << " ";
    std::cout << std::endl;
  }

  /*double frequency = 500;
  int samples = 10;
  RTDE rtde("127.0.0.1");
  rtde.connect();
  rtde.negotiateProtocolVersion();
  rtde.getControllerVersion();
  std::vector<std::string> variables = {"joint_temperatures", "target_q",    "actual_q",
                                        "actual_TCP_pose",    "safety_mode", "robot_mode"};
  rtde.sendOutputSetup(variables, frequency);
  rtde.sendStart();

  std::vector<int> durations;

  // Init Robot state
  std::shared_ptr<RobotState> robot_state = std::make_shared<RobotState>();



  // delay for a while
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // tell the other threads to stop
  done.set_value();

  std::cout << "Done\n";


  int i = 1;
  bool keep_running = true;
  while (keep_running)
  {
    if (i % frequency)
    {
      // Write to CSV
    }

    if (samples > 0 and i >= samples)
    {
      keep_running = false;
    }

    // Receive data
    auto start = high_resolution_clock::now();
    rtde.receiveData(robot_state);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    // std::cout << "Getting a sample took: " << duration.count() << "us" << std::endl;
    durations.push_back(duration.count());

    for (const auto &d : robot_state->getJoint_temperatures())
      std::cout << d << " ";
    std::cout << std::endl;
    i += 1;
  }

  double sum = std::accumulate(durations.begin(), durations.end(), 0);
  double average = sum / durations.size();
  std::cout << "Average sample acquisition rate: " << average << "us" << std::endl;

  rtde.sendPause();
  rtde.disconnect();*/

  return 0;
}