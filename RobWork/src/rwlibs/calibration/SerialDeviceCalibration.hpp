/*
 * SerialDeviceCalibration.hpp
 *
 *  Created on: Jul 4, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_SERIALDEVICECALIBRATION_HPP
#define RWLIBS_CALIBRATION_SERIALDEVICECALIBRATION_HPP

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/eigen/TransformAddons.hpp"

#include "CompositeCalibration.hpp"
#include "DHParameterCalibration.hpp"
//#include "EncoderParameterCalibration.hpp"
#include "FixedFrameCalibration.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>
#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {

class SerialDeviceCalibration: public CompositeCalibration<Calibration> {
public:
	typedef rw::common::Ptr<SerialDeviceCalibration> Ptr;

	SerialDeviceCalibration(rw::models::SerialDevice::Ptr serialDevice);

	SerialDeviceCalibration(rw::models::SerialDevice::Ptr serialDevice, FixedFrameCalibration::Ptr baseCalibration, FixedFrameCalibration::Ptr endCalibration,
			const CompositeCalibration<DHParameterCalibration>::Ptr& compositeDHParameterCalibration/*,
			 const CompositeCalibration<EncoderParameterCalibration>::Ptr& compositeEncoderParameterCalibration*/);

	virtual ~SerialDeviceCalibration();

	rw::models::SerialDevice::Ptr getDevice() const;

	FixedFrameCalibration::Ptr getBaseCalibration() const;

	FixedFrameCalibration::Ptr getEndCalibration() const;

	CompositeCalibration<DHParameterCalibration>::Ptr getCompositeDHParameterCalibration() const;

//	CompositeCalibration<EncoderParameterCalibration>::Ptr getCompositeEncoderParameterCalibration() const;

	static SerialDeviceCalibration::Ptr get(rw::models::SerialDevice::Ptr serialDevice);

	static SerialDeviceCalibration::Ptr get(const rw::common::PropertyMap& propertyMap);

	static void set(SerialDeviceCalibration::Ptr calibration, rw::models::SerialDevice::Ptr device);

	static void set(SerialDeviceCalibration::Ptr calibration, rw::common::PropertyMap& propertyMap);

private:
	rw::models::SerialDevice::Ptr _serialDevice;
	FixedFrameCalibration::Ptr _baseCalibration;
	FixedFrameCalibration::Ptr _endCalibration;
	CompositeCalibration<DHParameterCalibration>::Ptr _compositeDHParameterCalibration;
//	CompositeCalibration<EncoderParameterCalibration>::Ptr _compositeEncoderParameterCalibration;
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICECALIBRATION_HPP */
