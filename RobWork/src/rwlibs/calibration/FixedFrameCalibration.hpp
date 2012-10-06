/*
 * FrameCalibration.hpp
 *
 *  Created on: Aug 28, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_FRAMECALIBRATION_HPP_
#define RWLIBS_CALIBRATION_FRAMECALIBRATION_HPP_

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformAddons.hpp"

#include "Calibration.hpp"
#include <Eigen/Geometry>
#include <rw/kinematics.hpp>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

class FixedFrameCalibration: public Calibration {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<FixedFrameCalibration> Ptr;

	FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPreCorrection = true, const Eigen::Affine3d& transform = Eigen::Affine3d::Identity());

	virtual ~FixedFrameCalibration();

	rw::kinematics::FixedFrame::Ptr getFrame() const;

	Eigen::Affine3d getTransform() const;

	void setTransform(const Eigen::Affine3d& transform);

	bool isPreCorrection() const;

	void setLockedParameters(bool x, bool y, bool z, bool roll, bool pitch, bool yaw);

private:
	virtual void doApply();

	virtual void doRevert();

	virtual void doCorrect(rw::kinematics::State& state);

	virtual int doGetParameterCount() const;

	virtual Eigen::MatrixXd doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame, const rw::kinematics::State& state);

	virtual void doTakeStep(const Eigen::VectorXd& step);

private:
	rw::kinematics::FixedFrame::Ptr _frame;
	bool _isPreCorrection;
	Eigen::Affine3d _transform;
	Eigen::Matrix<int, 6, 1> _lockedParameters;
};

}
}

#endif /* RWLIBS_CALIBRATION_FRAMECALIBRATION_HPP_ */
