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

#include "SerialDeviceCalibration.hpp"
#include "SerialDeviceJacobian.hpp"
#include "SerialDevicePoseMeasurement.hpp"
#include "SerialDevicePoseMeasurementList.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

class SerialDeviceCalibrator {
public:
	typedef rw::common::Ptr<SerialDeviceCalibrator> Ptr;

	SerialDeviceCalibrator(rw::models::SerialDevice::Ptr device, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, DeviceJacobian::Ptr jacobian);

	virtual ~SerialDeviceCalibrator();

	rw::kinematics::Frame::Ptr getReferenceFrame() const;

	rw::kinematics::Frame::Ptr getMeasurementFrame() const;

	DeviceJacobian::Ptr getJacobian() const;

	void setMeasurementList(const SerialDevicePoseMeasurementList& measurements);

	void setWeight(bool weight);

	int getMinimumMeasurementCount() const;

	void calibrate(const rw::kinematics::State& state);

	void computeJacobian(Eigen::MatrixXd& jacobian, rw::kinematics::State state);

	void computeJacobian(Eigen::MatrixXd& jacobian, rw::kinematics::State state, const SerialDevicePoseMeasurementList& measurements);

	void computeResiduals(Eigen::VectorXd& residuals, rw::kinematics::State state);

	void computeResiduals(Eigen::VectorXd& residuals, rw::kinematics::State state, const SerialDevicePoseMeasurementList& measurements);

private:
	rw::models::SerialDevice::Ptr _device;
	rw::kinematics::Frame::Ptr _referenceFrame;
	rw::kinematics::Frame::Ptr _measurementFrame;
	DeviceJacobian::Ptr _jacobian;
	SerialDevicePoseMeasurementList _measurements;
	bool _weight;
	int _maxIterations;
	double _precision;
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICECALIBRATOR_HPP */
