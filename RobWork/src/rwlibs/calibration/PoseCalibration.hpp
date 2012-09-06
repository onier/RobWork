/*
 * PoseCalibration.hpp
 *
 *  Created on: Aug 30, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_POSECALIBRATION_HPP_
#define RWLIBS_CALIBRATION_POSECALIBRATION_HPP_

#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {

// TODO: Remove apply-concept, replace by enabled/disabled concept?
class PoseCalibration {
public:
	typedef rw::common::Ptr<PoseCalibration> Ptr;

	virtual ~PoseCalibration() {
	}

	virtual bool isEnabled() const = 0;

	virtual void apply() = 0;

	virtual void revert() = 0;

	virtual void correct(rw::kinematics::State& state) = 0;

	virtual bool isApplied() const = 0;
};

}
}

#endif /* RWLIBS_CALIBRATION_POSECALIBRATION_HPP_ */
