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

#include "Calibration.hpp"
#include "nlls/NLLSSystem.hpp"
#include "nlls/NLLSSolverLog.hpp"
#include "DevicePoseMeasurement.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

class SerialDeviceCalibrator: public NLLSSystem {
public:
	typedef rw::common::Ptr<SerialDeviceCalibrator> Ptr;

	SerialDeviceCalibrator(rw::models::SerialDevice::Ptr device, const rw::kinematics::State& state, rw::kinematics::Frame::Ptr referenceFrame,
			rw::kinematics::Frame::Ptr measurementFrame, Calibration::Ptr calibration);

	virtual ~SerialDeviceCalibrator();

	rw::kinematics::Frame::Ptr getReferenceFrame() const;

	void setReferenceFrame(rw::kinematics::Frame::Ptr referenceFrame);

	rw::kinematics::Frame::Ptr getMeasurementFrame() const;

	void setMeasurementFrame(rw::kinematics::Frame::Ptr measurementFrame);

	Calibration::Ptr getCalibration() const;

	unsigned int getMinimumMeasurementCount() const;

	const std::vector<DevicePoseMeasurement::Ptr>& getMeasurements() const;

	void addMeasurement(DevicePoseMeasurement::Ptr measurement);

	void addMeasurement(const rw::math::Q& q, const Pose6D<double>& pose, const Eigen::Matrix<double, 6, 6>& covarianceMatrix =
			Eigen::Matrix<double, 6, 6>::Identity());

	void setMeasurements(const std::vector<DevicePoseMeasurement::Ptr>& measurements);

	void setWeight(bool weight);

	NLLSSolverLog::Ptr getLog() const;

	void calibrate();

	virtual void computeJacobian(Eigen::MatrixXd& jacobian);

	void computeJacobian(Eigen::MatrixXd& jacobian, const std::vector<DevicePoseMeasurement::Ptr>& measurements);

	virtual void computeResiduals(Eigen::VectorXd& residuals);

	void computeResiduals(Eigen::VectorXd& residuals, const std::vector<DevicePoseMeasurement::Ptr>& measurements);

	virtual void takeStep(const Eigen::VectorXd& step);

private:
	rw::models::SerialDevice::Ptr _device;
	rw::kinematics::State _state;
	rw::kinematics::Frame::Ptr _referenceFrame;
	rw::kinematics::Frame::Ptr _measurementFrame;
	Calibration::Ptr _calibration;
	std::vector<DevicePoseMeasurement::Ptr> _measurements;
	bool _weight;
	NLLSSolverLog::Ptr _log;
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICECALIBRATOR_HPP */
