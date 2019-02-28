#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <rtde_control_interface.h>
#include <rtde_receive_interface.h>
namespace py = pybind11;

namespace rtde_control
{
PYBIND11_MODULE(rtde_control, m)
{
  m.doc() = "RTDE Control Interface";
  py::class_<RTDEControlInterface>(m, "RTDEControlInterface")
      .def(py::init<std::string>())
      .def("stopRobot", &RTDEControlInterface::stopRobot)
      .def("moveJ",
           (bool (RTDEControlInterface::*)(const std::vector<std::vector<double>> &path)) & RTDEControlInterface::moveJ,
           "moveJ with path")
      .def("moveJ", (bool (RTDEControlInterface::*)(const std::vector<double> &q, double speed, double acceleration)) &
                        RTDEControlInterface::moveJ,
           "moveJ without path")
      .def("moveJ_IK", &RTDEControlInterface::moveJ_IK)
      .def("moveL",
           (bool (RTDEControlInterface::*)(const std::vector<std::vector<double>> &path)) & RTDEControlInterface::moveL,
           "moveL with path")
      .def("moveL",
           (bool (RTDEControlInterface::*)(const std::vector<double> &pose, double speed, double acceleration)) &
               RTDEControlInterface::moveL,
           "moveL without path")
      .def("moveL_FK", &RTDEControlInterface::moveL_FK)
      .def("moveC", &RTDEControlInterface::moveC)
      .def("speedJ", &RTDEControlInterface::speedJ)
      .def("speedL", &RTDEControlInterface::speedL)
      .def("servoJ", &RTDEControlInterface::servoJ)
      .def("servoC", &RTDEControlInterface::servoC)
      .def("forceModeStart", &RTDEControlInterface::forceModeStart)
      .def("forceModeUpdate", &RTDEControlInterface::forceModeUpdate)
      .def("forceModeStop", &RTDEControlInterface::forceModeStop)
      .def("zeroFtSensor", &RTDEControlInterface::zeroFtSensor)
      .def("setStandardDigitalOut", &RTDEControlInterface::setStandardDigitalOut)
      .def("setToolDigitalOut", &RTDEControlInterface::setToolDigitalOut)
      .def("__repr__", [](const RTDEControlInterface &a)
           {
        return "<rtde_control.RTDEControlInterface>";
      });
}
};

namespace rtde_receive
{
PYBIND11_MODULE(rtde_receive, m)
{
  m.doc() = "RTDE Receive Interface";
  py::class_<RTDEReceiveInterface>(m, "RTDEReceiveInterface")
      .def(py::init<std::string>())
      .def("getTimestamp", &RTDEReceiveInterface::getTimestamp)
      .def("getTargetQ", &RTDEReceiveInterface::getTargetQ)
      .def("getTargetQd", &RTDEReceiveInterface::getTargetQd)
      .def("getTargetQdd", &RTDEReceiveInterface::getTargetQdd)
      .def("getTargetCurrent", &RTDEReceiveInterface::getTargetCurrent)
      .def("getTargetMoment", &RTDEReceiveInterface::getTargetMoment)
      .def("getActualQ", &RTDEReceiveInterface::getActualQ)
      .def("getActualQd", &RTDEReceiveInterface::getActualQd)
      .def("getActualCurrent", &RTDEReceiveInterface::getActualCurrent)
      .def("getJointControlOutput", &RTDEReceiveInterface::getJointControlOutput)
      .def("getActualTCPPose", &RTDEReceiveInterface::getActualTCPPose)
      .def("getActualTCPSpeed", &RTDEReceiveInterface::getActualTCPSpeed)
      .def("getActualTCPForce", &RTDEReceiveInterface::getActualTCPForce)
      .def("getTargetTCPPose", &RTDEReceiveInterface::getTargetTCPPose)
      .def("getTargetTCPSpeed", &RTDEReceiveInterface::getTargetTCPSpeed)
      .def("getActualDigitalInputBits", &RTDEReceiveInterface::getActualDigitalInputBits)
      .def("getJointTemperatures", &RTDEReceiveInterface::getJointTemperatures)
      .def("getActualExecutionTime", &RTDEReceiveInterface::getActualExecutionTime)
      .def("getRobotModes", &RTDEReceiveInterface::getRobotMode)
      .def("getJointMode", &RTDEReceiveInterface::getJointMode)
      .def("getSafetyMode", &RTDEReceiveInterface::getSafetyMode)
      .def("getActualToolAccelerometer", &RTDEReceiveInterface::getActualToolAccelerometer)
      .def("getSpeedScaling", &RTDEReceiveInterface::getSpeedScaling)
      .def("getTargetSpeedFraction", &RTDEReceiveInterface::getTargetSpeedFraction)
      .def("getActualMomentum", &RTDEReceiveInterface::getActualMomentum)
      .def("getActualMainVoltage", &RTDEReceiveInterface::getActualMainVoltage)
      .def("getActualRobotVoltage", &RTDEReceiveInterface::getActualRobotVoltage)
      .def("getActualRobotCurrent", &RTDEReceiveInterface::getActualRobotCurrent)
      .def("getActualJointVoltage", &RTDEReceiveInterface::getActualJointVoltage)
      .def("getActualDigitalOutputBits", &RTDEReceiveInterface::getActualDigitalOutputBits)
      .def("getRuntimeState", &RTDEReceiveInterface::getRuntimeState)
      .def("__repr__", [](const RTDEReceiveInterface &a)
           {
        return "<rtde_receive.RTDEReceiveInterface>";
      });
}
};