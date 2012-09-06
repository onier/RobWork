/*
 * DeviceCalibration.hpp
 *
 *  Created on: Aug 30, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_DEVICECALIBRATION_HPP_
#define RWLIBS_CALIBRATION_DEVICECALIBRATION_HPP_

#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {

class DeviceCalibration {
public:
	typedef rw::common::Ptr<DeviceCalibration> Ptr;

	virtual ~DeviceCalibration() {
	}

	virtual bool isEnabled() const = 0;

	virtual void apply() = 0;

	virtual void revert() = 0;

	virtual void correct(rw::kinematics::State& state) = 0;

	virtual bool isApplied() const = 0;
};

}
}

#endif /* RWLIBS_CALIBRATION_DEVICECALIBRATION_HPP_ */
