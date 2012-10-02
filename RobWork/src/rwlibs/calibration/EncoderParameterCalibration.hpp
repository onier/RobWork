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

namespace rwlibs {
namespace calibration {

class EncoderParameterCalibration: public Calibration {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<EncoderParameterCalibration> Ptr;

	EncoderParameterCalibration(rw::models::JointDevice::Ptr jointDevice, rw::models::Joint::Ptr joint);

	EncoderParameterCalibration(rw::models::JointDevice::Ptr jointDevice, rw::models::Joint::Ptr joint, const Eigen::VectorXd& correction);

	virtual ~EncoderParameterCalibration();

	rw::models::Joint::Ptr getJoint() const;

	void setLockedParameters(bool tau, bool sigma);

	QDomElement toXml(QDomDocument& document);

	static EncoderParameterCalibration::Ptr fromXml(const QDomElement& element, rw::kinematics::StateStructure::Ptr stateStructure, rw::models::JointDevice::Ptr jointDevice);

private:
	virtual void doApply();

	virtual void doRevert();

	virtual void doCorrect(rw::kinematics::State& state);

	virtual int doGetParameterCount() const;

	virtual Eigen::MatrixXd doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& state);

	virtual void doTakeStep(const Eigen::VectorXd& step);

	virtual int getCorrectionFunctionCount() const;

	virtual Eigen::VectorXd computeCorrectionFunctionVector(const double& q);

private:
	rw::models::JointDevice::Ptr _jointDevice;
	rw::models::Joint::Ptr _joint;
	Eigen::VectorXd _parameters;
	Eigen::VectorXi _lockedParameters;
	int _jointIndex;
};

}
}

#endif /* RWLIBS_CALIBRATION_ENCODERPARAMETERCALIBRATION_HPP_ */
