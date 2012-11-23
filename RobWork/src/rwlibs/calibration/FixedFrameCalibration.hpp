/*
 * FixedFrameCalibration.hpp
 *
 *  Created on: Aug 28, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_FIXEDFRAMECALIBRATION_HPP_
#define RWLIBS_CALIBRATION_FIXEDFRAMECALIBRATION_HPP_

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
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<FixedFrameCalibration> Ptr;

	FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame);

	FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPostCorrection);

	FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPostCorrection, const rw::math::Transform3D<>& transform);

	virtual ~FixedFrameCalibration();

	rw::kinematics::FixedFrame::Ptr getFrame() const;

	bool isPostCorrection() const;

	rw::math::Transform3D<> getCorrectionTransform() const;

	void setCorrectionTransform(const rw::math::Transform3D<>& correctionTransform);

private:
	virtual void doApply();

	virtual void doRevert();

	virtual void doCorrectState(rw::kinematics::State& state);

private:
	rw::kinematics::FixedFrame::Ptr _frame;
	bool _isPostCorrection;
	rw::math::Transform3D<> _correctionTransform;
	rw::math::Transform3D<> _originalTransform;
};

}
}

#endif /* RWLIBS_CALIBRATION_FIXEDFRAMECALIBRATION_HPP_ */
