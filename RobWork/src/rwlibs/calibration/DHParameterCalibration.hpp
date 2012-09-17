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
#include <QtXml/qdom.h>

namespace rwlibs {
namespace calibration {

class DHParameterCalibration: public Calibration {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<DHParameterCalibration> Ptr;

	DHParameterCalibration(rw::models::Joint::Ptr joint);

	DHParameterCalibration(rw::models::Joint::Ptr joint, const rw::models::DHParameterSet& correction);

	virtual ~DHParameterCalibration();

	rw::models::Joint::Ptr getJoint() const;

	rw::models::DHParameterSet getCorrection() const;

	void setEnabledParameters(bool a, bool length, bool alpha, bool angle);

	QDomElement toXml(QDomDocument& document);

	static DHParameterCalibration::Ptr fromXml(const QDomElement& element, rw::kinematics::StateStructure::Ptr stateStructure);

protected:
	virtual void doApply();

	virtual void doRevert();

	virtual void doCorrect(rw::kinematics::State& state);

	virtual int doGetParameterCount() const;

	virtual Eigen::MatrixXd doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& state);

	virtual void doStep(const Eigen::VectorXd& step);

private:
	rw::models::Joint::Ptr _joint;
	rw::models::DHParameterSet _correction;
	Eigen::Vector4i _enabledParameters;
};

}
}

#endif /* RWLIBS_CALIBRATION_DHPARAMETERCALIBRATION_HPP_ */
