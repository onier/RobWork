/*
 * FixedFrameJacobian.hpp
 *
 *  Created on: Nov 20, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_FIXEDFRAMEJACOBIAN_HPP_
#define RWLIBS_CALIBRATION_FIXEDFRAMEJACOBIAN_HPP_

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/eigen/EigenTransformPlugin.hpp"

#include "JacobianBase.hpp"
#include "FixedFrameCalibration.hpp"
#include <Eigen/Geometry>
#include <rw/kinematics.hpp>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

class FixedFrameJacobian: public JacobianBase {
public:
	enum PARAMETER {
		PARAMETER_X = 0,
		PARAMETER_Y = 1,
		PARAMETER_Z = 2,
		PARAMETER_ROLL = 3,
		PARAMETER_PITCH = 4,
		PARAMETER_YAW = 5
	};

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<FixedFrameJacobian> Ptr;

	FixedFrameJacobian(FixedFrameCalibration::Ptr calibration);

	virtual ~FixedFrameJacobian();

private:
	virtual Eigen::MatrixXd doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame, const rw::kinematics::State& state);

	virtual void doTakeStep(const Eigen::VectorXd& step);

private:
	FixedFrameCalibration::Ptr _calibration;
};

}
}

#endif /* RWLIBS_CALIBRATION_FIXEDFRAMEJACOBIAN_HPP_ */
