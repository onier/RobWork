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

#include "PoseCalibration.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>
#include <rw/models/DHParameterSet.hpp>
#include <QtXml/qdom.h>

namespace rwlibs {
namespace calibration {

class DHParameterCalibration: public PoseCalibration {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<DHParameterCalibration> Ptr;

	DHParameterCalibration(rw::models::Joint::Ptr joint);

	DHParameterCalibration(rw::models::Joint::Ptr joint, const rw::models::DHParameterSet& correction);

	virtual ~DHParameterCalibration();

	virtual bool isEnabled() const;

	virtual void apply();

	virtual void revert();

	virtual void correct(rw::kinematics::State& state);

	virtual bool isApplied() const;

	rw::models::Joint::Ptr getJoint() const;

	rw::models::DHParameterSet getCorrection() const;

	void correct(const rw::models::DHParameterSet& correction);

	QDomElement toXml(QDomDocument& document);

	static DHParameterCalibration::Ptr fromXml(const QDomElement& element, rw::kinematics::StateStructure::Ptr stateStructure);

private:
	bool _isEnabled;
	bool _isApplied;
	rw::models::Joint::Ptr _joint;
	rw::models::DHParameterSet _correction;
};

}
}

#endif /* RWLIBS_CALIBRATION_DHPARAMETERCALIBRATION_HPP_ */
