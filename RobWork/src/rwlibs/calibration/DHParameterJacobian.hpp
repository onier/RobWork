/*
 * DHParameterJacobian.hpp
 *
 *  Created on: Nov 20, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_DHPARAMETERJACOBIAN_HPP_
#define RWLIBS_CALIBRATION_DHPARAMETERJACOBIAN_HPP_

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/eigen/EigenTransformPlugin.hpp"

#include "DHParameterCalibration.hpp"
#include "JacobianBase.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>
#include <rw/models/DHParameterSet.hpp>

namespace rwlibs {
namespace calibration {

class DHParameterJacobian: public JacobianBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<DHParameterJacobian> Ptr;

	DHParameterJacobian(DHParameterCalibration::Ptr calibration);

	virtual ~DHParameterJacobian();

protected:
	virtual Eigen::MatrixXd doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame, const rw::kinematics::State& state);

private:
	DHParameterCalibration::Ptr _calibration;
};

}
}

#endif /* RWLIBS_CALIBRATION_DHPARAMETERJACOBIAN_HPP_ */
