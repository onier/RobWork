/*
 * EncoderParameterCalibration.hpp
 *
 *  Created on: Aug 28, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_ENCODERPARAMETERCALIBRATION_HPP_
#define RWLIBS_CALIBRATION_ENCODERPARAMETERCALIBRATION_HPP_

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformAddons.hpp"

#include "DeviceCalibration.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>
#include <QtXml/qdom.h>

namespace rwlibs {
namespace calibration {

class EncoderParameterCalibration: public DeviceCalibration {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<EncoderParameterCalibration> Ptr;

	EncoderParameterCalibration(rw::models::JointDevice::Ptr jointDevice, rw::models::Joint::Ptr joint, const Eigen::Vector2d& correction = Eigen::Vector2d::Zero());

	virtual ~EncoderParameterCalibration();

	virtual bool isEnabled() const;

	virtual void apply();

	virtual void revert();

	virtual void correct(rw::kinematics::State& state);

	virtual bool isApplied() const;

	rw::models::Joint::Ptr getJoint() const;

	Eigen::Vector2d getCorrection() const;

	void correct(const Eigen::Vector2d& correction);

	QDomElement toXml(QDomDocument& document);

	static EncoderParameterCalibration::Ptr fromXml(const QDomElement& element, rw::kinematics::StateStructure::Ptr stateStructure, rw::models::JointDevice::Ptr jointDevice);

private:
	bool _isEnabled;
	bool _isApplied;
	rw::models::JointDevice::Ptr _jointDevice;
	rw::models::Joint::Ptr _joint;
	Eigen::Vector2d _correction;
	int _jointNo;
};

}
}

#endif /* RWLIBS_CALIBRATION_ENCODERPARAMETERCALIBRATION_HPP_ */
