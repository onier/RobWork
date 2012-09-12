/*
 * DHParameterJacobian.hpp
 *
 *  Created on: Aug 30, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_DHPARAMETERJACOBIAN_HPP_
#define RWLIBS_CALIBRATION_DHPARAMETERJACOBIAN_HPP_

#include "DeviceJacobian.hpp"
#include "DHParameterCalibration.hpp"

namespace rwlibs {
namespace calibration {

class DHParameterJacobian: public DeviceJacobian {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<DHParameterJacobian> Ptr;

	DHParameterJacobian(DHParameterCalibration::Ptr calibration);

	virtual int getParameterCount() const;

	virtual Eigen::MatrixXd compute(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& state);

	virtual void step(const Eigen::VectorXd& step);

	void setEnabledParameters(bool a, bool length, bool alpha, bool angle);

private:
	DHParameterCalibration::Ptr _calibration;
	Eigen::Vector4i _enabledParameters;
};

}
}

#endif /* DHPARAMETERJACOBIAN_HPP_ */
