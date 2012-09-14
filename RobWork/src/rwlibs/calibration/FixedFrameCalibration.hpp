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
#include <QtXml/qdom.h>

namespace rwlibs {
namespace calibration {

class FixedFrameCalibration: public Calibration {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<FixedFrameCalibration> Ptr;

	FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPreCorrection = true, const Eigen::Affine3d& correction = Eigen::Affine3d::Identity());

	virtual ~FixedFrameCalibration();

	rw::kinematics::FixedFrame::Ptr getFrame() const;

	bool isPreCorrection() const;

	Eigen::Affine3d getCorrection() const;

	void setEnabledParameters(bool x, bool y, bool z, bool roll, bool pitch, bool yaw);

	QDomElement toXml(QDomDocument& document);

	static FixedFrameCalibration::Ptr fromXml(const QDomElement& element, rw::kinematics::StateStructure::Ptr stateStructure);

protected:
	virtual void doApply();

	virtual void doRevert();

	virtual void doCorrect(rw::kinematics::State& state);

	virtual int doGetParameterCount() const;

	virtual Eigen::MatrixXd doCompute(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& state);

	virtual void doStep(const Eigen::VectorXd& step);

private:
	rw::kinematics::FixedFrame::Ptr _frame;
	bool _isPreCorrection;
	Eigen::Affine3d _correction;
	Eigen::Matrix<int, 6, 1> _enabledParameters;
};

}
}

#endif /* RWLIBS_CALIBRATION_FRAMECALIBRATION_HPP_ */
