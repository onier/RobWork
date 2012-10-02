/*
 * DevicePoseMeasurement.hpp
 *
 *  Created on: Apr 14, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_DEVICEPOSEMEASUREMENT_HPP
#define RWLIBS_CALIBRATION_DEVICEPOSEMEASUREMENT_HPP

#include "Pose6D.hpp"
#include <rw/math.hpp>

namespace rwlibs {
namespace calibration {

class DevicePoseMeasurement {
public:
	typedef rw::common::Ptr<DevicePoseMeasurement> Ptr;

	DevicePoseMeasurement(const rw::math::Q& state, const Pose6D<double>& pose, const Eigen::Matrix<double, 6, 6>& covariance = Eigen::Matrix<double, 6, 6>::Identity());

	virtual ~DevicePoseMeasurement();

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

#endif /* RWLIBS_CALIBRATION_DEVICEPOSEMEASUREMENT_HPP */
