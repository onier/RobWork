/*
 * DHParameterCalibration.hpp
 *
 *  Created on: Aug 28, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_DHPARAMETERCALIBRATION_HPP_
#define RWLIBS_CALIBRATION_DHPARAMETERCALIBRATION_HPP_

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/eigen/EigenTransformPlugin.hpp"

#include "Calibration.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>
#include <rw/models/DHParameterSet.hpp>

namespace rwlibs {
namespace calibration {

class DHParameterCalibration: public Calibration {
public:
	enum PARAMETER {
		PARAMETER_A = 0,
		PARAMETER_B = 1,
		PARAMETER_D = 1,
		PARAMETER_B_D = 1,
		PARAMETER_ALPHA = 2,
		PARAMETER_BETA = 3,
		PARAMETER_THETA = 3,
		PARAMETER_BETA_THETA = 3
	};

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<DHParameterCalibration> Ptr;

	DHParameterCalibration(rw::models::Joint::Ptr joint, const Eigen::Vector4d& parameterSet = Eigen::Vector4d::Zero());

	virtual ~DHParameterCalibration();

	rw::models::Joint::Ptr getJoint() const;

	Eigen::Vector4d getCorrection() const;

	void setCorrection(const Eigen::Vector4d& correction);

private:
	virtual void doApply();

	virtual void doRevert();

	virtual void doCorrectState(rw::kinematics::State& state);

private:
	rw::models::Joint::Ptr _joint;
	Eigen::Vector4d _correction;
	rw::models::DHParameterSet _originalSet;
};

}
}

#endif /* RWLIBS_CALIBRATION_DHPARAMETERCALIBRATION_HPP_ */
