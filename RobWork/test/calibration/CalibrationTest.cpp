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

void printCalibration(rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibration);

BOOST_AUTO_TEST_CASE( RunCalibrationTest ) {
	std::string workCellFilePath(testFilePath() + "MARVIN/Marvin.wc.xml");
	std::string deviceName("UR1");
	std::string referenceFrameName("BB1Left");
	std::string measurementFilePath(testFilePath() + "MARVIN/Measurements400.xml");
	std::string calibrationFilePath(testFilePath() + "MARVIN/UR1.calibration.xml");

	// Load workcell.
	std::cout << "Loading work cell from file: \"" << workCellFilePath << "\"..";
	std::cout.flush();
	rw::models::WorkCell::Ptr workCell = rw::loaders::XMLRWLoader::load(workCellFilePath);
	rw::kinematics::State state = workCell->getDefaultState();
	std::cout << " Loaded." << std::endl;

	// Find device and frames.
	rw::models::SerialDevice::Ptr serialDevice = workCell->findDevice(deviceName).cast<rw::models::SerialDevice>();
	rw::kinematics::Frame::Ptr referenceFrame = workCell->findFrame(referenceFrameName);
	rw::kinematics::Frame::Ptr measurementFrame = serialDevice->getEnd();
	std::cout << "Device: " << serialDevice->getName() << ". Reference frame: " << referenceFrame->getName() << ". Measurement frame: "
			<< measurementFrame->getName() << "." << std::endl;

	// Load robot pose measurements from file.
	std::cout << "Loading measurement file: \"" << measurementFilePath << "\"..";
	std::cout.flush();
	rwlibs::calibration::SerialDevicePoseMeasurementList serialDevicePoseMeasurementList = rwlibs::calibration::SerialDevicePoseMeasurementList::load(
			measurementFilePath);
	std::cout << " Loaded " << serialDevicePoseMeasurementList.size() << " robot pose measurements." << std::endl;

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

	std::cout << "Calibrating device.. ";
	std::cout.flush();
	try {
		// Run calibrator.
		serialDeviceCalibrator->calibrate(state);
		std::cout << "Calibrated." << std::endl;

		// Save calibration.
		std::cout << "Saving calibration.. ";
		std::cout.flush();
		serialDeviceCalibration->save(calibrationFilePath);
		std::cout << "Saved." << std::endl;
	} catch (rw::common::Exception& ex) {
		RW_THROW(ex);
	}

//	// Print calibration results.
//	std::cout << std::endl;
//	std::cout << "-- Calibration data --" << std::endl;
//	printCalibration(serialDeviceCalibration);
//
//	// Print default device pose uncalibrated and calibrated.
//	std::cout << std::endl;
//	std::cout << "-- Default device pose --" << std::endl;
//	std::cout << "Uncalibrated: " << rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), serialDevice->getEnd(), state) << std::endl;
//
//	serialDeviceCalibration->apply();
//	serialDeviceCalibration->correct(state);
//
//	std::cout << "Calibrated: " << rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), serialDevice->getEnd(), state) << std::endl;
}

void printCalibration(rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibration) {
	rwlibs::calibration::Pose6D<double> poseBaseCorrection = rwlibs::calibration::Pose6D<double>(
			serialDeviceCalibration->getBaseCalibration()->getCorrection());
	rwlibs::calibration::Pose6D<double> poseEndCorrection = rwlibs::calibration::Pose6D<double>(serialDeviceCalibration->getEndCalibration()->getCorrection());
	std::cout << "Base: " << poseBaseCorrection.transpose() << " (" << poseBaseCorrection.norm() << ")" << std::endl;
	std::cout << "End: " << poseEndCorrection.transpose() << " (" << poseEndCorrection.norm() << ")" << std::endl;

	QList<rwlibs::calibration::DHParameterCalibration::Ptr> dhParameterCalibrations = serialDeviceCalibration->getDHParameterCalibrations();
	QListIterator<rwlibs::calibration::DHParameterCalibration::Ptr> itDh(dhParameterCalibrations);
	while (itDh.hasNext()) {
		rwlibs::calibration::DHParameterCalibration::Ptr dhParameterCalibration = itDh.next();
		rw::models::Joint::Ptr joint = dhParameterCalibration->getJoint();
		rw::models::DHParameterSet correction = dhParameterCalibration->getCorrection();
		if (correction.isParallel()) {
			std::cout << "DH (" << joint->getName() << "): [ a:\t" << correction.a() << "\talpha:\t" << correction.alpha() << "\tb:\t" << correction.b()
					<< "\tbeta:\t" << correction.beta() << " ]" << std::endl;
		} else {
			std::cout << "DH (" << joint->getName() << "): [ a:\t" << correction.a() << "\talpha:\t" << correction.alpha() << "\td:\t" << correction.d()
					<< "\ttheta:\t" << correction.theta() << " ]" << std::endl;
		}
	}

	QList<rwlibs::calibration::EncoderParameterCalibration::Ptr> encoderCalibrations = serialDeviceCalibration->getEncoderDecentralizationCalibrations();
	QListIterator<rwlibs::calibration::EncoderParameterCalibration::Ptr> itEncoder(encoderCalibrations);
	while (itEncoder.hasNext()) {
		rwlibs::calibration::EncoderParameterCalibration::Ptr encoderCalibration = itEncoder.next();
		std::cout << "Encoder (" << encoderCalibration->getJoint()->getName() << "): [ " << encoderCalibration->getCorrection().transpose() << " ]"
				<< std::endl;
	}
}
