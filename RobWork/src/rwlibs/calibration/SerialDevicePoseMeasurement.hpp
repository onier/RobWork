/*
 * SerialDevicePoseMeasurement.hpp
 *
 *  Created on: Apr 14, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENT_HPP
#define RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENT_HPP

#include "eigen/Pose6D.hpp"
#include <rw/math.hpp>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

class SerialDevicePoseMeasurement {
public:
	typedef rw::common::Ptr<SerialDevicePoseMeasurement> Ptr;

	SerialDevicePoseMeasurement(const rw::math::Q& q, const Pose6Dd& pose);

	SerialDevicePoseMeasurement(const rw::math::Q& q, const Pose6Dd& pose, const Eigen::Matrix<double, 6, 6>& covariance);

	virtual ~SerialDevicePoseMeasurement();

	rw::math::Q getQ() const;

	Pose6Dd getPose() const;

	Eigen::Matrix<double, 6, 6> getCovariance() const;

	bool hasCovariance() const;

private:
	rw::math::Q _q;
	Pose6Dd _pose;
	Eigen::Matrix<double, 6, 6> _covariance;
	bool _hasCovariance;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENT_HPP */