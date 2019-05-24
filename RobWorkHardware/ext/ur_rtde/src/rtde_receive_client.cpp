#include <ur_rtde/rtde_receive_interface.h>
#include <iostream>
#include <chrono>
#include <numeric>

using namespace ur_rtde;

int main(int argc, char *argv[])
{
  RTDEReceiveInterface rtde_receive("127.0.0.1");

  while (1)
  {
    std::cout << "Actual q is: " << std::endl;
    for (const auto &d : rtde_receive.getActualQ())
      std::cout << d << " ";
    std::cout << std::endl;
  }

  return 0;
}