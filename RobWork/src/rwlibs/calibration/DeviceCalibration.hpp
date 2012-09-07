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

	bool isEnabled() const {
		return _isEnabled;
	}

	void setEnabled(bool isEnabled) {
		_isEnabled = isEnabled;
	}

	bool isApplied() const {
		return _isApplied;
	}

	void apply() {
		if (!_isEnabled)
			RW_THROW("Not enabled.");
		if (_isApplied)
			RW_THROW("Already applied.");

		doApply();

		_isApplied = true;
	}

	virtual void revert() {
		if (!_isEnabled)
			RW_THROW("Not enabled.");
		if (!_isApplied)
			RW_THROW("Not applied.");

		doRevert();

		_isApplied = false;
	}

	virtual void correct(rw::kinematics::State& state) {
		if (!_isEnabled)
			RW_THROW("Not enabled.");
		if (!_isApplied)
			RW_WARN("Not applied.");

		doCorrect(state);
	}

protected:
	DeviceCalibration() :
			_isEnabled(true), _isApplied(false) {
	}

	virtual void doApply() = 0;

	virtual void doRevert() = 0;

	virtual void doCorrect(rw::kinematics::State& state) = 0;

private:
	bool _isEnabled;
	bool _isApplied;
};

}
}

#endif /* RWLIBS_CALIBRATION_DEVICECALIBRATION_HPP_ */
