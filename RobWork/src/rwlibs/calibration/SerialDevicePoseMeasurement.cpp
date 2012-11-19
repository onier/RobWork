/*
 * SerialDevicePoseMeasurement.cpp
 *
 *  Created on: Apr 14, 2012
 *      Author: bing
 */

#include "SerialDevicePoseMeasurement.hpp"

#include <rw/kinematics.hpp>
#include <QtCore>

namespace rwlibs {
namespace calibration {

SerialDevicePoseMeasurement::SerialDevicePoseMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform) :
	_q(q), _transform(transform), _hasCovariance(false) {

}

SerialDevicePoseMeasurement::SerialDevicePoseMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform, const Eigen::Matrix<double, 6, 6>& covariance) :
	_q(q), _transform(transform), _covariance(covariance), _hasCovariance(true) {

}

SerialDevicePoseMeasurement::~SerialDevicePoseMeasurement() {

}

const rw::math::Q& SerialDevicePoseMeasurement::getQ() const {
	return _q;
}

const rw::math::Transform3D<>& SerialDevicePoseMeasurement::getTransform() const {
	return _transform;
}

const Eigen::Matrix<double, 6, 6>& SerialDevicePoseMeasurement::getCovariance() const {
	return _covariance;
}

bool SerialDevicePoseMeasurement::hasCovariance() const {
	return _hasCovariance;
}

}
}
