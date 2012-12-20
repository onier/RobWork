/*
 * JointEncoderCalibration.hpp
 *
 *  Created on: Aug 28, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_JOINTENCODERCALIBRATION_HPP_
#define RWLIBS_CALIBRATION_JOINTENCODERCALIBRATION_HPP_

#include "CalibrationBase.hpp"
#include <rw/kinematics.hpp>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

class JointEncoderCalibration: public CalibrationBase {
public:
	static int PARAMETER_TAU, PARAMETER_SIGMA;

	typedef rw::common::Ptr<JointEncoderCalibration> Ptr;

	JointEncoderCalibration(rw::models::JointDevice::Ptr device, rw::models::Joint::Ptr joint);

	virtual ~JointEncoderCalibration();

	rw::models::JointDevice::Ptr getDevice() const;

	rw::models::Joint::Ptr getJoint() const;

private:
	virtual void doApply();

	virtual void doRevert();

private:
	rw::models::JointDevice::Ptr _device;
	rw::models::Joint::Ptr _joint;
};

}
}

#endif /* RWLIBS_CALIBRATION_JOINTENCODERCALIBRATION_HPP_ */
