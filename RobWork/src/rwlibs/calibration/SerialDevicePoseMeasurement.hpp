/*
 * SerialDevicePoseMeasurement.hpp
 *
 *  Created on: Apr 14, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENT_HPP
#define RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENT_HPP

#include "Pose6D.hpp"
#include <rw/math.hpp>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

class SerialDevicePoseMeasurement {
public:
	typedef rw::common::Ptr<SerialDevicePoseMeasurement> Ptr;

	SerialDevicePoseMeasurement(const rw::math::Q& q, const Pose6D<double>& pose, const Eigen::Matrix<double, 6, 6>& covariance = Eigen::Matrix<double, 6, 6>::Identity());

	virtual ~SerialDevicePoseMeasurement();

	rw::math::Q getQ() const;

	Pose6D<double> getPose() const;

	Eigen::Matrix<double, 6, 6> getCovariance() const;

private:
	rw::math::Q _q;
	Pose6D<double> _pose;
	Eigen::Matrix<double, 6, 6> _covariance;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENT_HPP */
