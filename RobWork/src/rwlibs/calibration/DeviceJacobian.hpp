/*
 * DeviceJacobian.hpp
 *
 *  Created on: Aug 28, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_DEVICEJACOBIAN_HPP_
#define RWLIBS_CALIBRATION_DEVICEJACOBIAN_HPP_

#include "DeviceCalibration.hpp"
#include <Eigen/Core>
#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {

class DeviceJacobian {
public:
	typedef rw::common::Ptr<DeviceJacobian> Ptr;

	virtual ~DeviceJacobian() {

	}

	virtual bool isEnabled() const {
		return _isEnabled;
	}

	virtual void setEnabled(bool isEnabled) {
		_isEnabled = isEnabled;
	}

	virtual DeviceCalibration::Ptr getCalibration() const = 0;

	virtual int getParameterCount() const = 0;

	virtual Eigen::MatrixXd compute(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
			const rw::kinematics::State& state) = 0;

	virtual void step(const Eigen::VectorXd& step) = 0;

protected:
	DeviceJacobian() :
			_isEnabled(true) {
	}

	bool _isEnabled;
};

}
}

#endif /* RWLIBS_CALIBRATION_DEVICEJACOBIAN_HPP_ */
