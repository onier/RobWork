/*
 * SerialDeviceCalibration.hpp
 *
 *  Created on: Jul 4, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_SERIALDEVICECALIBRATION_HPP
#define RWLIBS_CALIBRATION_SERIALDEVICECALIBRATION_HPP

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformAddons.hpp"

#include "DHParameterCalibration.hpp"
#include "EncoderParameterCalibration.hpp"
#include "FixedFrameCalibration.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>
#include <rw/models/DHParameterSet.hpp>
#include <rw/kinematics.hpp>
#include <QtXml/qdom.h>

namespace rwlibs {
namespace calibration {

class SerialDeviceCalibration: public Calibration {
public:
	typedef rw::common::Ptr<SerialDeviceCalibration> Ptr;

	SerialDeviceCalibration(rw::models::SerialDevice::Ptr serialDevice);

	SerialDeviceCalibration(rw::models::SerialDevice::Ptr serialDevice, FixedFrameCalibration::Ptr baseCalibration, FixedFrameCalibration::Ptr endCalibration, const std::vector<DHParameterCalibration::Ptr>& dhParameterCalibrations, const std::vector<EncoderParameterCalibration::Ptr>& encoderDecentralization);

	virtual ~SerialDeviceCalibration();

	rw::models::SerialDevice::Ptr getDevice() const;

	FixedFrameCalibration::Ptr getBaseCalibration() const;

	FixedFrameCalibration::Ptr getEndCalibration() const;

	std::vector<DHParameterCalibration::Ptr> getDHParameterCalibrations() const;

	std::vector<EncoderParameterCalibration::Ptr> getEncoderParameterCalibrations() const;

	void save(std::string fileName);

	static SerialDeviceCalibration::Ptr get(rw::models::SerialDevice::Ptr serialDevice);

	static SerialDeviceCalibration::Ptr get(const rw::common::PropertyMap& propertyMap);

	static void set(SerialDeviceCalibration::Ptr calibration, rw::models::SerialDevice::Ptr device);

	static void set(SerialDeviceCalibration::Ptr calibration, rw::common::PropertyMap& propertyMap);

	static SerialDeviceCalibration::Ptr load(rw::kinematics::StateStructure::Ptr stateStructure, rw::models::SerialDevice::Ptr device, std::string fileName);

protected:
	virtual void doApply();

	virtual void doRevert();

	virtual void doCorrect(rw::kinematics::State& state);

private:
	rw::models::SerialDevice::Ptr _serialDevice;
	FixedFrameCalibration::Ptr _baseCalibration;
	FixedFrameCalibration::Ptr _endCalibration;
	std::vector<DHParameterCalibration::Ptr> _dhParameterCalibrations;
	std::vector<EncoderParameterCalibration::Ptr> _encoderParameterCalibrations;
	std::vector<Calibration::Ptr> _calibrations;
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICECALIBRATION_HPP */
