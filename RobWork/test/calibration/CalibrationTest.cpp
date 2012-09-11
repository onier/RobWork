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
	rw::models::WorkCell::Ptr workCell = rw::loaders::XMLRWLoader::load(workCellFilePath);
	BOOST_REQUIRE_MESSAGE(!workCell.isNull(), "Work cell not loaded.");
	rw::kinematics::State state = workCell->getDefaultState();

	// Find device and frames.
	rw::models::SerialDevice::Ptr serialDevice = workCell->findDevice(deviceName).cast<rw::models::SerialDevice>();
	BOOST_REQUIRE_MESSAGE(!serialDevice.isNull(), "Device not found.");
	rw::kinematics::Frame::Ptr referenceFrame = workCell->findFrame(referenceFrameName);
	BOOST_REQUIRE_MESSAGE(!referenceFrame.isNull(), "Reference frame not found.");
	rw::kinematics::Frame::Ptr measurementFrame = serialDevice->getEnd();
	BOOST_REQUIRE_MESSAGE(!referenceFrame.isNull(), "Measurement frame not found.");

	// Find current calibration.
	rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibrationLast = rwlibs::calibration::SerialDeviceCalibration::get(serialDevice);
	BOOST_CHECK_MESSAGE(!serialDeviceCalibrationLast.isNull(), "Existing calibration not found.");
	serialDeviceCalibrationLast->revert();

	// Load robot pose measurements from file.
	rwlibs::calibration::SerialDevicePoseMeasurementList serialDevicePoseMeasurementList = rwlibs::calibration::SerialDevicePoseMeasurementList::load(
			measurementFilePath);
	BOOST_CHECK_MESSAGE(serialDevicePoseMeasurementList.size() == 400, "Measurement list does not contain 400 measurements.");

	// Initialize calibration, jacobian and calibrator.
	rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibration(
			rw::common::ownedPtr(new rwlibs::calibration::SerialDeviceCalibration(serialDevice)));

	rwlibs::calibration::SerialDeviceJacobian::Ptr serialDeviceJacobian(
			rw::common::ownedPtr(new rwlibs::calibration::SerialDeviceJacobian(serialDeviceCalibration)));
//	serialDeviceJacobian->getBaseJacobian()->setEnabled(false);
//	serialDeviceJacobian->getEndJacobian()->setEnabled(false);
//	serialDeviceJacobian->setDHParameterJacobiansEnabled(false);
//	serialDeviceJacobian->setEncoderDecentralizationJacobiansEnabled(false);

	rwlibs::calibration::SerialDeviceCalibrator::Ptr serialDeviceCalibrator(
			rw::common::ownedPtr(new rwlibs::calibration::SerialDeviceCalibrator(serialDevice, referenceFrame, measurementFrame, serialDeviceJacobian)));
	serialDeviceCalibrator->setMeasurementList(serialDevicePoseMeasurementList);
	serialDeviceCalibrator->setWeight(false);

	try {
		// Run calibrator.
		serialDeviceCalibrator->calibrate(state);
	} catch (rw::common::Exception& ex) {
		BOOST_FAIL(ex.getMessage());
	}

	// Save and load calibration.
	serialDeviceCalibration->save(calibrationFilePath);
	rwlibs::calibration::SerialDeviceCalibration::Ptr calibrationLoaded = rwlibs::calibration::SerialDeviceCalibration::load(workCell->getStateStructure(), serialDevice, calibrationFilePath);
	BOOST_CHECK_MESSAGE(!calibrationLoaded.isNull(), "Save and load failed.");
}
