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

#include "Calibration.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>
#include <QtXml/qdom.h>

namespace rwlibs {
namespace calibration {

class EncoderParameterCalibration: public Calibration {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<EncoderParameterCalibration> Ptr;

	EncoderParameterCalibration(rw::models::JointDevice::Ptr jointDevice, rw::models::Joint::Ptr joint, const Eigen::Vector2d& correction = Eigen::Vector2d::Zero());

	virtual ~EncoderParameterCalibration();

	rw::models::Joint::Ptr getJoint() const;

	Eigen::Vector2d getCorrection() const;

	void setEnabledParameters(bool tau, bool sigma);

	QDomElement toXml(QDomDocument& document);

	static EncoderParameterCalibration::Ptr fromXml(const QDomElement& element, rw::kinematics::StateStructure::Ptr stateStructure, rw::models::JointDevice::Ptr jointDevice);

protected:
	virtual void doApply();

	virtual void doRevert();

	virtual void doCorrect(rw::kinematics::State& state);

	virtual int doGetParameterCount() const;

	virtual Eigen::MatrixXd doCompute(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& state);

	virtual void doStep(const Eigen::VectorXd& step);

private:
	bool _isEnabled;
	bool _isApplied;
	rw::models::JointDevice::Ptr _jointDevice;
	rw::models::Joint::Ptr _joint;
	Eigen::Vector2d _correction;
	Eigen::Vector2i _enabledParameters;
	int _jointNo;
};

}
}

#endif /* RWLIBS_CALIBRATION_ENCODERPARAMETERCALIBRATION_HPP_ */
