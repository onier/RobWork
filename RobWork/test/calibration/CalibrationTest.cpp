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

using namespace rwlibs::calibration;

std::vector<SerialDevicePoseMeasurement> generateMeasurements(rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, unsigned int measurementCount, bool addNoise);

BOOST_AUTO_TEST_CASE( CalibratorTest ) {
	_CrtSetDbgFlag(0);

	const std::string testFilesPath = testFilePath();
	BOOST_REQUIRE(!testFilesPath.empty());

	const std::string workCellFilePath(testFilesPath + "calibration/Scene/SomeScene.wc.xml");
	const std::string deviceName("SomeDevice");
	const std::string referenceFrameName("SomeSensorFrame");
	const std::string measurementFrameName("SomeDevice.Marker");
	const std::string calibrationFilePath(testFilesPath + "calibration/SomeCalibration.xml");
	const int measurementCount = 40;

	// Load workcell.
	rw::models::WorkCell::Ptr workCell = rw::loaders::XMLRWLoader::load(workCellFilePath);
	BOOST_REQUIRE(!workCell.isNull());
	rw::kinematics::State state = workCell->getDefaultState();

	// Find device and frames.
	rw::models::Device::Ptr device = workCell->findDevice(deviceName);
	BOOST_REQUIRE(!device.isNull());
	rw::models::SerialDevice::Ptr serialDevice = device.cast<rw::models::SerialDevice>();
	rw::kinematics::Frame::Ptr referenceFrame = workCell->findFrame(referenceFrameName);
	BOOST_REQUIRE(!referenceFrame.isNull());
	rw::kinematics::Frame::Ptr measurementFrame = workCell->findFrame(measurementFrameName);
	BOOST_REQUIRE(!measurementFrame.isNull());

	// Find existing calibration.
	SerialDeviceCalibration::Ptr serialDeviceCalibrationExisting = SerialDeviceCalibration::get(serialDevice);
	//BOOST_CHECK(!serialDeviceCalibrationExisting.isNull());
	if (!serialDeviceCalibrationExisting.isNull())
		serialDeviceCalibrationExisting->revert();

	// Setup artificial calibration.
	SerialDeviceCalibration::Ptr artificialCalibration(rw::common::ownedPtr(new SerialDeviceCalibration(serialDevice)));
	artificialCalibration->getBaseCalibration()->setCorrectionTransform(rw::math::Transform3D<>(rw::math::Vector3D<>(0.07, 0.008, 0.009), rw::math::RPY<>(0.08, 0.007, 0.06)));
	artificialCalibration->getEndCalibration()->setCorrectionTransform(rw::math::Transform3D<>(rw::math::Vector3D<>(0.01, 0.002, 0.003), rw::math::RPY<>(0.04, 0.005, 0.06)));
	std::vector<DHParameterCalibration::Ptr> artificialInternalLinkCalibrations =
		artificialCalibration->getInternalLinkCalibration()->getCalibrations();
	for (unsigned int calibrationIndex = 0; calibrationIndex < artificialInternalLinkCalibrations.size(); calibrationIndex++) {
		if (artificialInternalLinkCalibrations[calibrationIndex]->isParameterEnabled(DHParameterCalibration::PARAMETER_A))
			artificialInternalLinkCalibrations[calibrationIndex]->setParameterValue(DHParameterCalibration::PARAMETER_A, 0.003);
		if (artificialInternalLinkCalibrations[calibrationIndex]->isParameterEnabled(DHParameterCalibration::PARAMETER_B))
			artificialInternalLinkCalibrations[calibrationIndex]->setParameterValue(DHParameterCalibration::PARAMETER_B, -0.001);
		if (artificialInternalLinkCalibrations[calibrationIndex]->isParameterEnabled(DHParameterCalibration::PARAMETER_D))
			artificialInternalLinkCalibrations[calibrationIndex]->setParameterValue(DHParameterCalibration::PARAMETER_D, -0.001);
		if (artificialInternalLinkCalibrations[calibrationIndex]->isParameterEnabled(DHParameterCalibration::PARAMETER_ALPHA))
			artificialInternalLinkCalibrations[calibrationIndex]->setParameterValue(DHParameterCalibration::PARAMETER_ALPHA, -0.002);
		if (artificialInternalLinkCalibrations[calibrationIndex]->isParameterEnabled(DHParameterCalibration::PARAMETER_BETA))
			artificialInternalLinkCalibrations[calibrationIndex]->setParameterValue(DHParameterCalibration::PARAMETER_BETA, 0.004);
		if (artificialInternalLinkCalibrations[calibrationIndex]->isParameterEnabled(DHParameterCalibration::PARAMETER_THETA))
			artificialInternalLinkCalibrations[calibrationIndex]->setParameterValue(DHParameterCalibration::PARAMETER_THETA, 0.004);
	}

	artificialCalibration->apply();

	// Load robot pose measurements from file.
	const std::vector<SerialDevicePoseMeasurement> measurements = generateMeasurements(serialDevice, referenceFrame, measurementFrame, state, measurementCount, false);
	BOOST_CHECK_EQUAL(measurements.size(), measurementCount);

	artificialCalibration->revert();

	// Initialize calibration, jacobian and calibrator.
	SerialDeviceCalibration::Ptr calibration(rw::common::ownedPtr(new SerialDeviceCalibration(serialDevice)));
	SerialDeviceJacobian::Ptr jacobian(rw::common::ownedPtr(new SerialDeviceJacobian(calibration)));
	SerialDeviceCalibrator::Ptr calibrator(rw::common::ownedPtr(new SerialDeviceCalibrator(serialDevice, referenceFrame, measurementFrame, calibration, jacobian)));
	calibrator->setMeasurements(measurements);

	try {
		// Run calibrator.
		calibrator->calibrate(state);
	} catch (rw::common::Exception& ex) {
		BOOST_ERROR(ex.getMessage());
	}

	const int iterationCount = calibrator->getSolver()->getIterationLogs().back().getIterationNumber();
	BOOST_CHECK_EQUAL(iterationCount, 6);

	// Verify that the calibration match the artificial calibration.
	{
		const rw::math::Transform3D<> artificialBaseCorrectionTransform = artificialCalibration->getBaseCalibration()->getCorrectionTransform();
		const rw::math::Transform3D<> baseCorrectionTransform = calibration->getBaseCalibration()->getCorrectionTransform();
		double baseDistanceError = (artificialBaseCorrectionTransform.P() - baseCorrectionTransform.P()).norm2();
		BOOST_CHECK_SMALL(baseDistanceError, 10e-5);
		double baseAngleError = rw::math::EAA<>(artificialBaseCorrectionTransform.R() * rw::math::inverse(baseCorrectionTransform.R())).angle();
		BOOST_CHECK_SMALL(baseAngleError, 10e-5);
	}
	{
		const rw::math::Transform3D<> artificialEndCorrectionTransform = artificialCalibration->getEndCalibration()->getCorrectionTransform();
		const rw::math::Transform3D<> endCorrectionTransform = calibration->getEndCalibration()->getCorrectionTransform();
		double endDistanceError = (artificialEndCorrectionTransform.P() - endCorrectionTransform.P()).norm2();
		BOOST_CHECK_SMALL(endDistanceError, 10e-5);
		double endAngleError = rw::math::EAA<>(artificialEndCorrectionTransform.R() * rw::math::inverse(endCorrectionTransform.R())).angle();
		BOOST_CHECK_SMALL(endAngleError, 10e-5);
	}
	std::vector<DHParameterCalibration::Ptr> internalLinkCalibrations =
		calibration->getInternalLinkCalibration()->getCalibrations();
	for (unsigned int calibrationIndex = 0; calibrationIndex < internalLinkCalibrations.size(); calibrationIndex++) {
		for (int parameterIndex = 0; parameterIndex < artificialInternalLinkCalibrations[calibrationIndex]->getParameterCount(); parameterIndex++) {
			if (artificialInternalLinkCalibrations[calibrationIndex]->isParameterEnabled(parameterIndex)) {
				double artificialInternalLinkParameterValue = artificialInternalLinkCalibrations[calibrationIndex]->getParameterValue(parameterIndex);
				double internalLinkParameterValue = internalLinkCalibrations[calibrationIndex]->getParameterValue(parameterIndex);
				double internalLinkParameterError = internalLinkParameterValue - artificialInternalLinkParameterValue;
				BOOST_CHECK_SMALL(internalLinkParameterError, 10e-5);
			}
		}
	}

	calibration->apply();

	// Verify that calibration fits measurements.
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		serialDevice->setQ(measurements[measurementIndex].getQ(), state);

		const rw::math::Transform3D<> measurementTransform = measurements[measurementIndex].getTransform();
		const rw::math::Transform3D<> modelTransform = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state);

		double measurementDistanceError = (modelTransform.P() - measurementTransform.P()).norm2();
		BOOST_CHECK_SMALL(measurementDistanceError, 10e-5);
		double measurementAngleError = rw::math::EAA<>(modelTransform.R() * rw::math::inverse(measurementTransform.R())).angle();
		BOOST_CHECK_SMALL(measurementAngleError, 10e-5);
	}

	calibration->revert();

	XmlCalibrationSaver::save(calibration, calibrationFilePath);

	/*SerialDeviceCalibration::Ptr calibrationLoaded = XmlCalibrationLoader::load(
	workCell->getStateStructure(), serialDevice, calibrationFilePath);

	calibrationLoaded->apply();

	// Verify that the loaded calibration match the artificial calibration.
	BOOST_CHECK(Eigen::Affine3d(calibrationLoaded->getBaseCalibration()->getCorrectionTransform()).isApprox(Eigen::Affine3d(artificialCalibration->getBaseCalibration()->getCorrectionTransform()), 10e-5));
	BOOST_CHECK(Eigen::Affine3d(calibrationLoaded->getEndCalibration()->getCorrectionTransform()).isApprox(Eigen::Affine3d(artificialCalibration->getEndCalibration()->getCorrectionTransform()), 10e-5));
	std::vector<DHParameterCalibration::Ptr> internalLinkCalibrationsLoaded =
	calibrationLoaded->getInternalLinkCalibration()->getCalibrations();
	for (unsigned int calibrationIndex = 0; calibrationIndex < internalLinkCalibrationsLoaded.size(); calibrationIndex++)
	for (int parameterIndex = 0; parameterIndex < 4; parameterIndex++)
	if (artificialInternalLinkCalibrations[calibrationIndex]->isParameterEnabled(parameterIndex)) {
	double diff = abs(abs(internalLinkCalibrationsLoaded[calibrationIndex]->getParameterValue(parameterIndex)) - abs(artificialInternalLinkCalibrations[calibrationIndex]->getParameterValue(parameterIndex)));
	BOOST_CHECK_CLOSE(diff, 0.0, 10e-5);
	}

	// Verify that loaded calibration fits measurements.
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
	serialDevice->setQ(measurements[measurementIndex].getQ(), state);

	const rw::math::Transform3D<> tfmMeasurement = measurements[measurementIndex].getTransform();
	const rw::math::Transform3D<> tfmModel = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state);
	const rw::math::Transform3D<> tfmError(tfmModel.P() - tfmMeasurement.P(), tfmModel.R() * rw::math::inverse(tfmMeasurement.R()));

	double measurementDistanceError = (tfmModel.P() - tfmMeasurement.P()).norm2();
	BOOST_CHECK_SMALL(measurementDistanceError, 10e-5);
	double measurementAngleError = rw::math::EAA<>(tfmModel.R() * rw::math::inverse(tfmMeasurement.R())).angle();
	BOOST_CHECK_SMALL(measurementAngleError, 10e-5);
	}*/
}

std::vector<SerialDevicePoseMeasurement> generateMeasurements(rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, unsigned int measurementCount, bool addNoise) {
	MultivariateNormalDistribution<double, 6> mvnd(time(0));

	std::vector<SerialDevicePoseMeasurement> measurements;
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		rw::math::Q q = rw::math::Math::ranQ(serialDevice->getBounds());
		serialDevice->setQ(q, state);

		rw::math::Transform3D<> transform = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state);
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
			transform.P() = rw::math::Vector3D<>(mvndVector(0), mvndVector(1), mvndVector(2)) + transform.P();
			transform.R() = rw::math::RPY<>(mvndVector(3), mvndVector(4), mvndVector(5)).toRotation3D() * transform.R();
		}

		measurements.push_back(SerialDevicePoseMeasurement(q, transform, covariance));
	}

	return measurements;
}
