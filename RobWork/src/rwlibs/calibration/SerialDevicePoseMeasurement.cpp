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

SerialDevicePoseMeasurement::SerialDevicePoseMeasurement(const rw::math::Q& q, const rw::math::Pose6D<>& pose) :
	_q(q), _pose(pose), _hasCovariance(false) {

}

SerialDevicePoseMeasurement::SerialDevicePoseMeasurement(const rw::math::Q& q, const rw::math::Pose6D<>& pose, const Eigen::Matrix<double, 6, 6>& covariance) :
	_q(q), _pose(pose), _covariance(covariance), _hasCovariance(true) {

}

SerialDevicePoseMeasurement::~SerialDevicePoseMeasurement() {

}

const rw::math::Q& SerialDevicePoseMeasurement::getQ() const {
	return _q;
}

const rw::math::Pose6D<>& SerialDevicePoseMeasurement::getPose() const {
	return _pose;
}

const Eigen::Matrix<double, 6, 6>& SerialDevicePoseMeasurement::getCovariance() const {
	return _covariance;
}

bool SerialDevicePoseMeasurement::hasCovariance() const {
	return _hasCovariance;
}

}
}
