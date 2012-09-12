/*
 * Calibration.hpp
 *
 *  Created on: Aug 30, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_CALIBRATION_HPP_
#define RWLIBS_CALIBRATION_CALIBRATION_HPP_

#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {

class Calibration {
public:
	typedef rw::common::Ptr<Calibration> Ptr;

	virtual ~Calibration() {
	}

	bool isEnabled() const {
		return _isEnabled;
	}

	void setEnabled(bool isEnabled) {
		if (_isApplied)
			RW_THROW("Already applied.");

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
	Calibration() :
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

#endif /* RWLIBS_CALIBRATION_CALIBRATION_HPP_ */
