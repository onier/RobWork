/*
 * CalibrationTest.cpp
 *
 *  Created on: Sep 7, 2012
 *      Author: bing
 */

#include "../TestSuiteConfig.hpp"

#include <rw/models.hpp>
#include <rw/kinematics.hpp>
#include <rw/loaders.hpp>
#include <rwlibs/calibration.hpp>

BOOST_AUTO_TEST_CASE( CalibrationTest ) {
	const std::string workCellFilePath(testFilePath() + "calibration/Scene/SomeScene.wc.xml");
	const std::string deviceName("SomeDevice");
	const std::string referenceFrameName("SomeSensorFrame");
	const std::string measurementFilePath(testFilePath() + "calibration/SomeMeasurements.xml");
	const std::string calibrationFilePath(testFilePath() + "calibration/SomeSerialDeviceCalibration.xml");

	// Load workcell.
	BOOST_TEST_CHECKPOINT("Loading work cell");
	rw::models::WorkCell::Ptr workCell = rw::loaders::XMLRWLoader::load(workCellFilePath);
	BOOST_REQUIRE_MESSAGE(!workCell.isNull(), "Work cell not loaded.");
	rw::kinematics::State state = workCell->getDefaultState();

	// Find device and frames.
	BOOST_TEST_CHECKPOINT("Finding device");
	rw::models::SerialDevice::Ptr device = workCell->findDevice(deviceName).cast<rw::models::SerialDevice>();
	BOOST_REQUIRE_MESSAGE(!device.isNull(), "Device not found.");
	BOOST_TEST_CHECKPOINT("Finding reference frame");
	rw::kinematics::Frame::Ptr referenceFrame = workCell->findFrame(referenceFrameName);
	BOOST_REQUIRE_MESSAGE(!referenceFrame.isNull(), "Reference frame not found.");
	BOOST_TEST_CHECKPOINT("Finding measurement frame");
	rw::kinematics::Frame::Ptr measurementFrame = device->getEnd();
	BOOST_REQUIRE_MESSAGE(!referenceFrame.isNull(), "Measurement frame not found.");

	// Find current calibration.
	BOOST_TEST_CHECKPOINT("Getting existing calibration");
	rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibrationLast = rwlibs::calibration::SerialDeviceCalibration::get(device);
	BOOST_CHECK_MESSAGE(!serialDeviceCalibrationLast.isNull(), "Existing calibration not found.");
	serialDeviceCalibrationLast->revert();

	// Load robot pose measurements from file.
	BOOST_TEST_CHECKPOINT("Loading measurements");
	std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> measurements = rwlibs::calibration::XmlMeasurementFile::load(measurementFilePath);
	BOOST_CHECK_MESSAGE(measurements.size() == 400, "Measurement list does not contain 400 measurements.");

	// Initialize calibration, jacobian and calibrator.
	BOOST_TEST_CHECKPOINT("Initializing calibration");
	rwlibs::calibration::SerialDeviceCalibration::Ptr calibration(rw::common::ownedPtr(new rwlibs::calibration::SerialDeviceCalibration(device)));
//	calibration->getCompositeDHParameterCalibration()->setEnabled(false);
//	calibration->getCompositeDHParameterCalibration()->getCalibrations()[0]->lockParameter(rwlibs::calibration::DHParameterCalibration::PARAMETER_A);

	BOOST_TEST_CHECKPOINT("Initializing calibrator");
	rwlibs::calibration::SerialDeviceCalibrator::Ptr calibrator(
			rw::common::ownedPtr(new rwlibs::calibration::SerialDeviceCalibrator(device, state, referenceFrame, measurementFrame, calibration)));
	calibrator->setMeasurements(measurements);
//	calibrator->setWeighted(false);

	try {
		// Run calibrator.
		BOOST_TEST_CHECKPOINT("Calibrating");
		calibrator->calibrate();
	} catch (rw::common::Exception& ex) {
		BOOST_FAIL(ex.getMessage());
	}

	// Save and load calibration.
	BOOST_TEST_CHECKPOINT("Saving and loading calibration");
	rwlibs::calibration::XmlCalibrationSaver::save(calibration, calibrationFilePath);
	rwlibs::calibration::SerialDeviceCalibration::Ptr calibrationLoaded = rwlibs::calibration::XmlCalibrationLoader::load(calibrationFilePath,
			workCell->getStateStructure(), device);
	BOOST_CHECK_MESSAGE(!calibrationLoaded.isNull(), "Save and load failed.");
}
