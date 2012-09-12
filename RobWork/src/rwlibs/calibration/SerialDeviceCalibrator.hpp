/*
 * SerialDeviceCalibrator.hpp
 *
 *  Created on: Feb 15, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_SERIALDEVICECALIBRATOR_HPP
#define RWLIBS_CALIBRATION_SERIALDEVICECALIBRATOR_HPP

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformAddons.hpp"

#include "DeviceJacobian.hpp"
#include "IterativeSolver.hpp"
#include "SerialDevicePoseMeasurementList.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

class SerialDeviceCalibrator: public IterativeSolver {
public:
	typedef rw::common::Ptr<SerialDeviceCalibrator> Ptr;

	SerialDeviceCalibrator(rw::models::SerialDevice::Ptr device, const rw::kinematics::State& state, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, DeviceJacobian::Ptr jacobian);

	virtual ~SerialDeviceCalibrator();

	rw::kinematics::Frame::Ptr getReferenceFrame() const;

	rw::kinematics::Frame::Ptr getMeasurementFrame() const;

	DeviceJacobian::Ptr getJacobian() const;

	void setMeasurementList(const SerialDevicePoseMeasurementList& measurements);

	void setWeight(bool weight);

	int getMinimumMeasurementCount() const;

	void calibrate();

	virtual void computeJacobian(Eigen::MatrixXd& jacobian);

	void computeJacobian(Eigen::MatrixXd& jacobian, const SerialDevicePoseMeasurementList& measurements);

	virtual void computeResiduals(Eigen::VectorXd& residuals);

	void computeResiduals(Eigen::VectorXd& residuals, const SerialDevicePoseMeasurementList& measurements);

	virtual void takeStep(const Eigen::VectorXd& step);

private:
	rw::models::SerialDevice::Ptr _device;
	rw::kinematics::State _state;
	rw::kinematics::Frame::Ptr _referenceFrame;
	rw::kinematics::Frame::Ptr _measurementFrame;
	DeviceJacobian::Ptr _jacobian;
	SerialDevicePoseMeasurementList _measurements;
	bool _weight;
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICECALIBRATOR_HPP */
