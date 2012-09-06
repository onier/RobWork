/*
 * SerialDeviceJacobian.hpp
 *
 *  Created on: Aug 30, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_SERIALDEVICEJACOBIAN_HPP_
#define RWLIBS_CALIBRATION_SERIALDEVICEJACOBIAN_HPP_

#include "DeviceJacobian.hpp"
#include "DHParameterJacobian.hpp"
#include "EncoderParameterJacobian.hpp"
#include "FixedFrameJacobian.hpp"
#include "SerialDeviceCalibration.hpp"

namespace rwlibs {
namespace calibration {

class SerialDeviceJacobian: public DeviceJacobian {
public:
	typedef rw::common::Ptr<SerialDeviceJacobian> Ptr;

	SerialDeviceJacobian(SerialDeviceCalibration::Ptr calibration);

	virtual ~SerialDeviceJacobian();

	virtual DeviceCalibration::Ptr getCalibration() const;

	virtual int getParameterCount() const;

	virtual Eigen::MatrixXd compute(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& state);

	virtual void step(const Eigen::VectorXd& step);

	FixedFrameJacobian::Ptr getBaseJacobian() const;

	FixedFrameJacobian::Ptr getEndJacobian() const;

	QList<DHParameterJacobian::Ptr> getDHParameterJacobians() const;

	void setDHParameterJacobiansEnabled(bool isEnabled);

	QList<EncoderParameterJacobian::Ptr> getEncoderDecentralizationJacobians() const;

	void setEncoderDecentralizationJacobiansEnabled(bool isEnabled);

private:
	SerialDeviceCalibration::Ptr _calibration;
	FixedFrameJacobian::Ptr _baseJacobian;
	FixedFrameJacobian::Ptr _endJacobian;
	QList<DHParameterJacobian::Ptr> _dhParameterJacobians;
	QList<EncoderParameterJacobian::Ptr> _encoderDecentralizationJacobians;
	QList<DeviceJacobian::Ptr> _jacobians;
};

}
}


#endif /* RWLIBS_CALIBRATION_SERIALDEVICEJACOBIAN_HPP_ */
