/*
 * SerialDevicePoseMeasurement.hpp
 *
 *  Created on: Apr 14, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENT_HPP
#define RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENT_HPP

#include <rw/math.hpp>
#include <rw/models.hpp>

#include <Eigen/Core>

namespace rwlibs {
namespace calibration {

class SerialDevicePoseMeasurement {
public:
	typedef rw::common::Ptr<SerialDevicePoseMeasurement> Ptr;

	SerialDevicePoseMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform);

	SerialDevicePoseMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform, const Eigen::Matrix<double, 6, 6>& covariance);

	virtual ~SerialDevicePoseMeasurement();

	const rw::math::Q& getQ() const;

	const rw::math::Transform3D<>& getTransform() const;

	const Eigen::Matrix<double, 6, 6>& getCovariance() const;

	bool hasCovariance() const;

private:
	rw::math::Q _q;
	rw::math::Transform3D<> _transform;
	Eigen::Matrix<double, 6, 6> _covariance;
	bool _hasCovariance;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENT_HPP */
