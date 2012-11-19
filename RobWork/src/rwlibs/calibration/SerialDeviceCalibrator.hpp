/*
 * SerialDeviceCalibrator.hpp
 *
 *  Created on: Feb 15, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_SERIALDEVICECALIBRATOR_HPP
#define RWLIBS_CALIBRATION_SERIALDEVICECALIBRATOR_HPP

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/eigen/EigenTransformPlugin.hpp"

#include "nlls/NLLSSystem.hpp"
#include "nlls/NLLSSolver.hpp"
#include "nlls/NLLSSolverLog.hpp"
#include "Calibration.hpp"
#include "SerialDevicePoseMeasurement.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

class SerialDeviceCalibrator: NLLSSystem {
public:
	typedef rw::common::Ptr<SerialDeviceCalibrator> Ptr;

	SerialDeviceCalibrator(rw::models::SerialDevice::Ptr device, rw::kinematics::Frame::Ptr referenceFrame,
			rw::kinematics::Frame::Ptr measurementFrame, Calibration::Ptr calibration);

	virtual ~SerialDeviceCalibrator();

	rw::kinematics::Frame::Ptr getReferenceFrame() const;

	void setReferenceFrame(rw::kinematics::Frame::Ptr referenceFrame);

	rw::kinematics::Frame::Ptr getMeasurementFrame() const;

	void setMeasurementFrame(rw::kinematics::Frame::Ptr measurementFrame);

	Calibration::Ptr getCalibration() const;

	unsigned int getMinimumMeasurementCount() const;

	const std::vector<SerialDevicePoseMeasurement::Ptr>& getMeasurements() const;

	void addMeasurement(SerialDevicePoseMeasurement::Ptr measurement);

	void addMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform, const Eigen::Matrix<double, 6, 6>& covarianceMatrix =
			Eigen::Matrix<double, 6, 6>::Identity());

	void setMeasurements(const std::vector<SerialDevicePoseMeasurement::Ptr>& measurements);

	bool isWeightingEnabled() const;

	void setWeightingEnabled(bool isWeightingEnabled);

	void calibrate(const rw::kinematics::State& state);

	NLLSSolverLog::Ptr getSolverLog() const;

	Eigen::MatrixXd estimateCovariance() const;

private:
	virtual void computeJacobian(Eigen::MatrixXd& jacobian);

	virtual void computeResiduals(Eigen::VectorXd& residuals);

	virtual void takeStep(const Eigen::VectorXd& step);

private:
	rw::models::SerialDevice::Ptr _device;
	rw::kinematics::State _state;
	rw::kinematics::Frame::Ptr _referenceFrame;
	rw::kinematics::Frame::Ptr _measurementFrame;
	Calibration::Ptr _calibration;
	std::vector<SerialDevicePoseMeasurement::Ptr> _measurements;
	bool _isWeightingEnabled;
	NLLSSolver::Ptr _solver;
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICECALIBRATOR_HPP */
