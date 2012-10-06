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

SerialDevicePoseMeasurement::SerialDevicePoseMeasurement(const rw::math::Q& q, const Pose6Dd& pose) :
	_q(q), _pose(pose), _hasCovariance(false) {

}

SerialDevicePoseMeasurement::SerialDevicePoseMeasurement(const rw::math::Q& q, const Pose6Dd& pose, const Eigen::Matrix<double, 6, 6>& covariance) :
	_q(q), _pose(pose), _covariance(covariance), _hasCovariance(true) {

}

SerialDevicePoseMeasurement::~SerialDevicePoseMeasurement() {

}

rw::math::Q SerialDevicePoseMeasurement::getQ() const {
	return _q;
}

Pose6Dd SerialDevicePoseMeasurement::getPose() const {
	return _pose;
}

Eigen::Matrix<double, 6, 6> SerialDevicePoseMeasurement::getCovariance() const {
	return _covariance;
}

bool SerialDevicePoseMeasurement::hasCovariance() const {
	return _hasCovariance;
}

}
}
