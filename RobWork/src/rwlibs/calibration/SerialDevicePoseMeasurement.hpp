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
#include <QtCore>
#include <QtXml/qdom.h>

namespace rwlibs {
namespace calibration {

class SerialDevicePoseMeasurement {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SerialDevicePoseMeasurement(const rw::math::Q& state, const Pose6D<double>& pose, const Eigen::Matrix<double, 6, 6>& covariance = Eigen::Matrix<double, 6, 6>::Identity());

	SerialDevicePoseMeasurement(QDomElement& element);

	virtual ~SerialDevicePoseMeasurement();

	rw::math::Q getQ() const;

	Pose6D<double> getPose() const;

	Eigen::Matrix<double, 6, 6> getCovariance() const;

	virtual QDomElement toXml(QDomDocument& document) const;

private:
	rw::math::Q _q;
	Pose6D<double> _pose;
	Eigen::Matrix<double, 6, 6> _covariance;
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENT_HPP */
