#include <iostream>
#include <rwhw/universalrobots_rtde/URRTDE.hpp>

using namespace rw;
using namespace rwhw;

int main(int argc, char** argv)
{
    rwhw::URRTDE ur_rtde("localhost");
    rw::math::Q actual_q = ur_rtde.getActualQ();
    std::cout << "Actual q: " << std::endl;
    std::cout << actual_q << std::endl;

    rw::math::Transform3D<> actual_tcp_pose = ur_rtde.getActualTCPPose();
    std::cout << "Actual tcp pose: " << std::endl;
    std::cout << actual_tcp_pose << std::endl;

    rw::math::Q new_q(actual_q);
    new_q[0] += rw::math::Pi/2;

    std::cout << "Moving to: " << new_q << " using moveJ" << std::endl;
    ur_rtde.moveJ(new_q, 0.5, 0.2);
    std::cout << "Moving back to: " << actual_q << " using moveJ" << std::endl;
    ur_rtde.moveJ(actual_q, 0.5, 0.2);

    double velocity = 0.5;
    double acceleration = 0.2;
    rw::math::Q q1(9, actual_q[0], actual_q[1], actual_q[2], actual_q[3], actual_q[4], actual_q[5], velocity, acceleration, 0.0);
    rw::math::Q q2(9, new_q[0], new_q[1], new_q[2], new_q[3], new_q[4], new_q[5], velocity, acceleration, 0.35);
    rw::math::Q q3(9, new_q[0], new_q[1], new_q[2], new_q[3], new_q[4], new_q[5], velocity, acceleration, 0.0);
    q3[2] += rw::math::Pi/4;

    rw::trajectory::QPath q_path;
    q_path.push_back(q1);
    q_path.push_back(q2);
    q_path.push_back(q3);

    std::cout << "Moving according to QPath using moveJ" << std::endl;
    ur_rtde.moveJ(q_path);

    std::cout << "Moving to: " << new_q << " using moveJ" << std::endl;
    ur_rtde.moveJ(new_q, 0.5, 0.2);
    std::cout << "Moving back to: " << actual_q << " using moveJ" << std::endl;
    ur_rtde.moveJ(actual_q, 0.5, 0.2);

    ur_rtde.stopRobot();

    return 0;
}
