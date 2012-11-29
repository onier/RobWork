/*
 * SerialDeviceJacobian.hpp
 *
 *  Created on: Jul 4, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_SERIALDEVICEJACOBIAN_HPP
#define RWLIBS_CALIBRATION_SERIALDEVICEJACOBIAN_HPP

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/eigen/EigenTransformPlugin.hpp"

#include "CompositeJacobian.hpp"
#include "DHParameterJacobian.hpp"
#include "FixedFrameJacobian.hpp"
#include "SerialDeviceCalibration.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>
#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {

class SerialDeviceJacobian: public CompositeJacobian<Jacobian> {
public:
	typedef rw::common::Ptr<SerialDeviceJacobian> Ptr;

	SerialDeviceJacobian(SerialDeviceCalibration::Ptr calibration);

	virtual ~SerialDeviceJacobian();

	FixedFrameJacobian::Ptr getBaseJacobian() const;

	FixedFrameJacobian::Ptr getEndJacobian() const;

	CompositeJacobian<DHParameterJacobian>::Ptr getInternalLinkJacobian() const;

private:
	FixedFrameJacobian::Ptr _baseJacobian;
	FixedFrameJacobian::Ptr _endJacobian;
	CompositeJacobian<DHParameterJacobian>::Ptr _internalLinkJacobian;
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICEJACOBIAN_HPP */
