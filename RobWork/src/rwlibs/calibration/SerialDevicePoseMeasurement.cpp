/*
 * SerialDevicePoseMeasurement.cpp
 *
 *  Created on: Apr 14, 2012
 *      Author: bing
 */

#include "SerialDevicePoseMeasurement.hpp"

#include <QtCore>

namespace rwlibs {
namespace calibration {

SerialDevicePoseMeasurement::SerialDevicePoseMeasurement(const rw::math::Q& q, const Pose6D<double>& pose, const Eigen::Matrix<double, 6, 6>& covariance) :
	_q(q), _pose(pose), _covariance(covariance) {

}

SerialDevicePoseMeasurement::~SerialDevicePoseMeasurement() {

}

rw::math::Q SerialDevicePoseMeasurement::getQ() const {
	return _q;
}

Pose6D<double> SerialDevicePoseMeasurement::getPose() const {
	return _pose;
}

Eigen::Matrix<double, 6, 6> SerialDevicePoseMeasurement::getCovariance() const {
	return _covariance;
}

}
}
