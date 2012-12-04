/*
 * DHParameterCalibration.hpp
 *
 *  Created on: Aug 28, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_DHPARAMETERCALIBRATION_HPP_
#define RWLIBS_CALIBRATION_DHPARAMETERCALIBRATION_HPP_

#include "CalibrationBase.hpp"
#include <rw/models.hpp>
#include <rw/models/DHParameterSet.hpp>

namespace rwlibs {
namespace calibration {

class DHParameterCalibration: public CalibrationBase {
public:
	static int PARAMETER_A, PARAMETER_B, PARAMETER_D, PARAMETER_ALPHA, PARAMETER_BETA, PARAMETER_THETA;

	typedef rw::common::Ptr<DHParameterCalibration> Ptr;

	DHParameterCalibration(rw::models::Joint::Ptr joint);

	virtual ~DHParameterCalibration();

	rw::models::Joint::Ptr getJoint() const;

private:
	virtual void doApply();

	virtual void doRevert();

private:
	rw::models::Joint::Ptr _joint;
	rw::models::DHParameterSet _originalSet;
	bool _isParallel;
};

}
}

#endif /* RWLIBS_CALIBRATION_DHPARAMETERCALIBRATION_HPP_ */
