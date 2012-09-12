/*
 * EncoderParameterJacobian.hpp
 *
 *  Created on: Aug 30, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_ENCODERPARAMETERJACOBIAN_HPP_
#define RWLIBS_CALIBRATION_ENCODERPARAMETERJACOBIAN_HPP_

#include "DeviceJacobian.hpp"
#include "EncoderParameterCalibration.hpp"
#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {

class EncoderParameterJacobian: public DeviceJacobian {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<EncoderParameterJacobian> Ptr;

	EncoderParameterJacobian(EncoderParameterCalibration::Ptr calibration, rw::models::JointDevice::Ptr jointDevice);

	virtual int getParameterCount() const;

	virtual Eigen::MatrixXd compute(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& state);

	virtual void step(const Eigen::VectorXd& step);

	void setEnabledParameters(bool tau, bool sigma);

private:
	EncoderParameterCalibration::Ptr _calibration;
	rw::models::JointDevice::Ptr _jointDevice;
	rw::models::Joint::Ptr _joint;
	Eigen::Vector2i _enabledParameters;
	int _jointNo;
};

}
}

#endif /* ENCODERDECENTRALIZATIONJACOBIAN_HPP_ */
