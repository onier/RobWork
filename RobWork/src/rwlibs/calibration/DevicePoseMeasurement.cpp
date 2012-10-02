/*
 * DevicePoseMeasurement.cpp
 *
 *  Created on: Apr 14, 2012
 *      Author: bing
 */

#include "DevicePoseMeasurement.hpp"

#include <QtCore>

namespace rwlibs {
namespace calibration {

DevicePoseMeasurement::DevicePoseMeasurement(const rw::math::Q& q, const Pose6D<double>& pose, const Eigen::Matrix<double, 6, 6>& covariance) :
	_q(q), _pose(pose), _covariance(covariance) {

}

DevicePoseMeasurement::~DevicePoseMeasurement() {

}

rw::math::Q DevicePoseMeasurement::getQ() const {
	return _q;
}

Pose6D<double> DevicePoseMeasurement::getPose() const {
	return _pose;
}

Eigen::Matrix<double, 6, 6> DevicePoseMeasurement::getCovariance() const {
	return _covariance;
}

}
}
