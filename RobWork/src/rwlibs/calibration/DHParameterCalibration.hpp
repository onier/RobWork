/*
 * DHParameterCalibration.hpp
 *
 *  Created on: Aug 28, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_DHPARAMETERCALIBRATION_HPP_
#define RWLIBS_CALIBRATION_DHPARAMETERCALIBRATION_HPP_

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformAddons.hpp"

#include "Calibration.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>
#include <rw/models/DHParameterSet.hpp>

namespace rwlibs {
namespace calibration {

class DHParameterCalibration: public Calibration {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<DHParameterCalibration> Ptr;

	DHParameterCalibration(rw::models::Joint::Ptr joint);

	DHParameterCalibration(rw::models::Joint::Ptr joint, const rw::models::DHParameterSet& dhParameterSet);

	virtual ~DHParameterCalibration();

	rw::models::Joint::Ptr getJoint() const;

	rw::models::DHParameterSet getCorrection() const;

	void setLockedParameters(bool a, bool length, bool alpha, bool angle);

private:
	virtual void doApply();

	virtual void doRevert();

	virtual void doCorrect(rw::kinematics::State& state);

	virtual int doGetParameterCount() const;

	virtual Eigen::MatrixXd doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame, const rw::kinematics::State& state);

	virtual void doTakeStep(const Eigen::VectorXd& step);

private:
	rw::models::Joint::Ptr _joint;
	rw::models::DHParameterSet _dhParameterSet;
	Eigen::Vector4i _lockedParameters;
};

}
}

#endif /* RWLIBS_CALIBRATION_DHPARAMETERCALIBRATION_HPP_ */
