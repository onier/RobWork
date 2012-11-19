/*
 * FixedFrameCalibration.hpp
 *
 *  Created on: Aug 28, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_FRAMECALIBRATION_HPP_
#define RWLIBS_CALIBRATION_FRAMECALIBRATION_HPP_

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/eigen/EigenTransformPlugin.hpp"

#include "Calibration.hpp"
#include <Eigen/Geometry>
#include <rw/kinematics.hpp>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

class FixedFrameCalibration: public Calibration {
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

	typedef rw::common::Ptr<FixedFrameCalibration> Ptr;

	FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame);

	FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPostCorrection);

	FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPostCorrection, const rw::math::Transform3D<>& transform);

	virtual ~FixedFrameCalibration();

	rw::kinematics::FixedFrame::Ptr getFrame() const;

	bool isPostCorrection() const;

	rw::math::Transform3D<> getCorrection() const;

	void setCorrection(const rw::math::Transform3D<>& transform);

	bool isParameterLocked(int parameterIndex);

	void setParameterLocked(int parameterIndex, bool isLocked);

private:
	virtual void doApply();

	virtual void doRevert();

	virtual void doCorrect(rw::kinematics::State& state);

	virtual int doGetParameterCount() const;

	virtual Eigen::MatrixXd doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame, const rw::kinematics::State& state);

	virtual void doTakeStep(const Eigen::VectorXd& step);

private:
	rw::kinematics::FixedFrame::Ptr _frame;
	bool _isPostCorrection;
	rw::math::Transform3D<> _correction;
	Eigen::Matrix<int, 6, 1> _lockedParameters;
};

}
}

#endif /* RWLIBS_CALIBRATION_FRAMECALIBRATION_HPP_ */
