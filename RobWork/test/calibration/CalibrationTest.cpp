/*
* CalibrationTest.cpp
*
*  Created on: Sep 7, 2012
*      Author: bing
*/

#include "../TestSuiteConfig.hpp"
#include "MultivariateNormalDistribution.hpp"
#include <rw/models.hpp>
#include <rw/kinematics.hpp>
#include <rw/loaders.hpp>
#include <rwlibs/calibration.hpp>

std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> generateMeasurements(rw::models::SerialDevice::Ptr serialDevice,
	rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, unsigned int measurementCount,
	bool addNoise);

BOOST_AUTO_TEST_CASE( CalibrationTest ) {
	const std::string testFilesPath(testFilePath());
	BOOST_REQUIRE_MESSAGE(!testFilesPath.empty(), "Test suite not initialized.");

	const std::string workCellFilePath(testFilesPath + "calibration/Scene/SomeScene.wc.xml");
	const std::string deviceName("SomeDevice");
	const std::string referenceFrameName("SomeSensorFrame");
	const std::string measurementFrameName("SomeDevice.Marker");
	const std::string calibrationFilePath(testFilesPath + "calibration/SomeCalibration.xml");
	const unsigned int measurementCount = 40;

	// Load workcell.
	BOOST_TEST_CHECKPOINT("Loading work cell");
	rw::models::WorkCell::Ptr workCell = rw::loaders::XMLRWLoader::load(workCellFilePath);
	BOOST_REQUIRE_MESSAGE(!workCell.isNull(), "Work cell not loaded.");
	rw::kinematics::State state = workCell->getDefaultState();

	// Find device and frames.
	BOOST_TEST_CHECKPOINT("Finding device");
	rw::models::Device::Ptr device = workCell->findDevice(deviceName);
	BOOST_REQUIRE_MESSAGE(!device.isNull(), "Device not found.");
	rw::models::SerialDevice::Ptr serialDevice = device.cast<rw::models::SerialDevice>();
	BOOST_TEST_CHECKPOINT("Finding reference frame");
	rw::kinematics::Frame::Ptr referenceFrame = workCell->findFrame(referenceFrameName);
	BOOST_REQUIRE_MESSAGE(!referenceFrame.isNull(), "Reference frame not found.");
	BOOST_TEST_CHECKPOINT("Finding measurement frame");
	rw::kinematics::Frame::Ptr measurementFrame = workCell->findFrame(measurementFrameName);
	BOOST_REQUIRE_MESSAGE(!measurementFrame.isNull(), "Measurement frame not found.");

	// Find existing calibration.
	BOOST_TEST_CHECKPOINT("Getting existing calibration");
	rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibrationExisting = rwlibs::calibration::SerialDeviceCalibration::get(serialDevice);
	BOOST_CHECK_MESSAGE(!serialDeviceCalibrationExisting.isNull(), "Existing serial calibration not found.");
	if (!serialDeviceCalibrationExisting.isNull())
		serialDeviceCalibrationExisting->revert();

	// Setup artificial calibration.
	BOOST_TEST_CHECKPOINT("Setting up artificial calibration");
	rwlibs::calibration::SerialDeviceCalibration::Ptr artificialCalibration(
		rw::common::ownedPtr(new rwlibs::calibration::SerialDeviceCalibration(serialDevice)));

	artificialCalibration->getBaseCalibration()->setCorrection(rw::math::Transform3D<>(rw::math::Vector3D<>(0.07, 0.008, 0.009), rw::math::RPY<>(0.08, 0.007, 0.06)));
	artificialCalibration->getEndCalibration()->setCorrection(rw::math::Transform3D<>(rw::math::Vector3D<>(0.01, 0.002, 0.003), rw::math::RPY<>(0.04, 0.005, 0.06)));
	std::vector<rwlibs::calibration::DHParameterCalibration::Ptr> artificialDhParameterCalibrations =
		artificialCalibration->getCompositeDHParameterCalibration()->getCalibrations();
	for (unsigned int calibrationIndex = 0; calibrationIndex < artificialDhParameterCalibrations.size(); calibrationIndex++)
		artificialDhParameterCalibrations[calibrationIndex]->setCorrection(Eigen::Vector4d(0.003, 0.0, -0.002, 0.0));

	BOOST_TEST_CHECKPOINT("Applying artificial calibration");
	artificialCalibration->apply();

	// Load robot pose measurements from file.
	BOOST_TEST_CHECKPOINT("Generating measurements");
	const std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> measurements = generateMeasurements(serialDevice, referenceFrame, measurementFrame,
		state, measurementCount, false);
	BOOST_CHECK_MESSAGE(measurements.size() == measurementCount, "Measurement generation failed.");

	BOOST_TEST_CHECKPOINT("Reverting artificial calibration");
	artificialCalibration->revert();

	// Initialize calibration, jacobian and calibrator.
	BOOST_TEST_CHECKPOINT("Initializing calibration");
	rwlibs::calibration::SerialDeviceCalibration::Ptr calibration(rw::common::ownedPtr(new rwlibs::calibration::SerialDeviceCalibration(serialDevice)));

	BOOST_TEST_CHECKPOINT("Initializing calibrator");
	rwlibs::calibration::SerialDeviceCalibrator::Ptr calibrator(
		rw::common::ownedPtr(new rwlibs::calibration::SerialDeviceCalibrator(serialDevice, referenceFrame, measurementFrame, calibration)));
	calibrator->setMeasurements(measurements);
	//	calibrator->setWeighted(false);

	try {
		// Run calibrator.
		BOOST_TEST_CHECKPOINT("Calibrating");
		calibrator->calibrate(state);
	} catch (rw::common::Exception& ex) {
		BOOST_ERROR(ex.getMessage());
	}

	// Verify that the calibration match the artificial calibration.
	BOOST_CHECK_MESSAGE(calibration->getBaseCalibration()->getCorrection().isApprox(artificialCalibration->getBaseCalibration()->getCorrection(), 10e-5),
		"Base calibration failed.");
	BOOST_CHECK_MESSAGE(calibration->getEndCalibration()->getCorrection().isApprox(artificialCalibration->getEndCalibration()->getCorrection(), 10e-5),
		"End calibration failed.");
	std::vector<rwlibs::calibration::DHParameterCalibration::Ptr> dhParameterCalibrations =
		calibration->getCompositeDHParameterCalibration()->getCalibrations();
	for (unsigned int calibrationIndex = 0; calibrationIndex < dhParameterCalibrations.size(); calibrationIndex++)
		BOOST_CHECK_MESSAGE(
		dhParameterCalibrations[calibrationIndex]->getCorrection().isApprox(artificialDhParameterCalibrations[calibrationIndex]->getCorrection(), 10e-5),
		"DH calibration failed.");

	BOOST_TEST_CHECKPOINT("Applying calibration");
	calibration->apply();

	// Verify that calibration fits measurements.
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		BOOST_TEST_CHECKPOINT("Updating state");
		serialDevice->setQ(measurements[measurementIndex]->getQ(), state);

		BOOST_TEST_CHECKPOINT("Computing measurement error");
		const Eigen::Affine3d tfmMeasurement(measurements[measurementIndex]->getPose().toTransform3D());
		const Eigen::Affine3d tfmModel = Eigen::Affine3d(rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state));
		const Eigen::Affine3d tfmError = tfmModel.difference(tfmMeasurement);

		BOOST_CHECK_MESSAGE(tfmError.isApprox(Eigen::Affine3d::Identity(), 10e-5), "Measurement error is non-zero.");
	}

	BOOST_TEST_CHECKPOINT("Reverting calibration");
	calibration->revert();

	BOOST_TEST_CHECKPOINT("Saving calibration");
	rwlibs::calibration::XmlCalibrationSaver::save(calibration, calibrationFilePath);

	BOOST_TEST_CHECKPOINT("Loading calibration");
	rwlibs::calibration::SerialDeviceCalibration::Ptr calibrationLoaded = rwlibs::calibration::XmlCalibrationLoader::load(calibrationFilePath,
		workCell->getStateStructure(), serialDevice);

	BOOST_TEST_CHECKPOINT("Applying loaded calibration");
	calibrationLoaded->apply();

	// Verify that the loaded calibration match the artificial calibration.
	BOOST_CHECK_MESSAGE(
		calibrationLoaded->getBaseCalibration()->getCorrection().isApprox(artificialCalibration->getBaseCalibration()->getCorrection(), 10e-5),
		"Loading base calibration failed.");
	BOOST_CHECK_MESSAGE(
		calibrationLoaded->getEndCalibration()->getCorrection().isApprox(artificialCalibration->getEndCalibration()->getCorrection(), 10e-5),
		"Loading end calibration failed.");
	std::vector<rwlibs::calibration::DHParameterCalibration::Ptr> dhParameterCalibrationsLoaded =
		calibrationLoaded->getCompositeDHParameterCalibration()->getCalibrations();
	for (unsigned int calibrationIndex = 0; calibrationIndex < dhParameterCalibrations.size(); calibrationIndex++)
		BOOST_CHECK_MESSAGE(
		dhParameterCalibrationsLoaded[calibrationIndex]->getCorrection().isApprox(artificialDhParameterCalibrations[calibrationIndex]->getCorrection(), 10e-5),
		"Loading DH calibration failed.");

	// Verify that loaded calibration fits measurements.
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		BOOST_TEST_CHECKPOINT("Updating state");
		serialDevice->setQ(measurements[measurementIndex]->getQ(), state);

		BOOST_TEST_CHECKPOINT("Computing measurement error");
		const Eigen::Affine3d tfmMeasurement(measurements[measurementIndex]->getPose().toTransform3D());
		const Eigen::Affine3d tfmModel = Eigen::Affine3d(rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state));
		const Eigen::Affine3d tfmError = tfmModel.difference(tfmMeasurement);

		BOOST_CHECK_MESSAGE(tfmError.isApprox(Eigen::Affine3d::Identity(), 10e-5), "Loaded measurement error is non-zero.");
	}
}

std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> generateMeasurements(rw::models::SerialDevice::Ptr serialDevice,
	rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, unsigned int measurementCount,
	bool addNoise) {
		MultivariateNormalDistribution<double, 6> mvnd(time(0));

		std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> measurements(measurementCount);
		for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
			rw::math::Q q = rw::math::Math::ranQ(serialDevice->getBounds());
			serialDevice->setQ(q, state);

			Eigen::Affine3d transform(rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state));
			Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
			if (addNoise) {
				Eigen::Matrix<double, 6, 6> random = Eigen::Matrix<double, 6, 6>::Random();
				covariance = random.transpose() * random;
				covariance.block<3, 3>(0, 0) /= 50.0;
				covariance.block<3, 3>(3, 3) /= 5.0;
				covariance.block<3, 3>(3, 0) /= 1000.0;
				covariance.block<3, 3>(0, 3) /= 1000.0;
				covariance /= 10e14;

				Eigen::Matrix<double, 6, 1> mvndVector = mvnd.draw(covariance);
				Eigen::Affine3d noise(rw::math::Transform3D<>(rw::math::Vector3D<>(mvndVector(0), mvndVector(1), mvndVector(2)), rw::math::RPY<>(mvndVector(3), mvndVector(4), mvndVector(5))));
				transform.linear() = noise.linear() * transform.linear();
				transform.translation() = noise.translation() + transform.translation();
			}

			rw::math::Pose6D<> noisyPose(transform);
			measurements[measurementIndex] = rw::common::ownedPtr(new rwlibs::calibration::SerialDevicePoseMeasurement(q, noisyPose, covariance));
		}

		return measurements;
}
