#include "MultivariateNormalDistribution.hpp"
#include <rw/common.hpp>
#include <rw/loaders.hpp>
#include <rwlibs/calibration.hpp>

boost::program_options::options_description optionsDescription("Options");
boost::program_options::positional_options_description positionalOptionsDescription;
boost::program_options::variables_map variablesMap;

std::string workCellFilePath;
std::string deviceName;
std::string referenceFrameName;
std::string measurementFrameName;
std::string measurementFilePath;
std::string calibrationFilePath;
bool isWeightingEnabled;
bool isBaseCalibrationEnabled;
bool isEndCalibrationEnabled;
bool isLinkCalibrationEnabled;

rw::models::WorkCell::Ptr workCell;
rw::kinematics::State state;
rw::models::Device::Ptr device;
rw::models::SerialDevice::Ptr serialDevice;
rw::kinematics::Frame::Ptr referenceFrame;
rw::kinematics::Frame::Ptr measurementFrame;

int parseArguments(int argumentCount, char** arguments);
void printHelp();
void printMeasurements(const std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr>& measurements, rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibration);
void printSolverLog(rwlibs::calibration::NLLSSolverLog::Ptr solverLog);
void printCalibrationSummary(rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibration);

int main(int argumentCount, char** arguments) {
	if (int result = parseArguments(argumentCount, arguments) < 1)
		return result;

	std::cout << "Initializing calibration tool:" << std::endl;

	// Load workcell.
	std::cout << "\tLoading work cell [ " << workCellFilePath << " ].. ";
	std::cout.flush();
	workCell = rw::loaders::XMLRWLoader::load(workCellFilePath);
	if (workCell.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Loaded [ " << workCell->getName() << " ]." << std::endl;
	state = workCell->getDefaultState();

	// Find device and frames.
	std::cout << "\tFinding device [ " << deviceName << " ].. ";
	std::cout.flush();
	device = deviceName.empty() ? workCell->getDevices().front() : workCell->findDevice(deviceName);
	if (device.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Found [ " << device->getName() << " ]." << std::endl;
	serialDevice = device.cast<rw::models::SerialDevice>();

	std::cout << "\tFinding reference frame [ " << referenceFrameName << " ].. ";
	std::cout.flush();
	referenceFrame = referenceFrameName.empty() ? workCell->findFrame("WORLD") : workCell->findFrame(referenceFrameName);
	if (referenceFrame.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Found [ " << referenceFrame->getName() << " ]." << std::endl;

	std::cout << "\tFinding measurement frame [ " << measurementFrameName << " ].. ";
	std::cout.flush();
	measurementFrame = measurementFrameName.empty() ? device->getEnd() : workCell->findFrame(measurementFrameName);
	if (measurementFrame.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Found [ " << measurementFrame->getName() << " ]." << std::endl;

	// Load robot pose measurements from file.
	std::cout << "\tLoading measurements [ " << measurementFilePath << " ].. ";
	std::cout.flush();
	std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> measurements = rwlibs::calibration::XmlMeasurementFile::load(measurementFilePath);
	std::cout << "Loaded " << measurements.size() << " measurements." << std::endl;

	// Disable existing calibration if one exist.
	std::cout << "\tFinding existing calibration.. ";
	std::cout.flush();
	rwlibs::calibration::SerialDeviceCalibration::Ptr calibrationExisting = rwlibs::calibration::SerialDeviceCalibration::get(serialDevice);
	if (calibrationExisting.isNull()) {
		std::cout << "Not found." << std::endl;
	} else {
		std::cout << "Found." << std::endl;

		std::cout << "\t\tDisabling existing calibration.. ";
		std::cout.flush();
		calibrationExisting->revert();
		std::cout << "Disabled." << std::endl;
	}

	// Initialize calibration, jacobian and calibrator.
	std::cout << "\tInitializing calibration.. ";
	std::cout.flush();
	rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibration = rw::common::ownedPtr(new rwlibs::calibration::SerialDeviceCalibration(serialDevice));
	serialDeviceCalibration->getBaseCalibration()->setLocked(!isBaseCalibrationEnabled);
	serialDeviceCalibration->getEndCalibration()->setLocked(!isEndCalibrationEnabled);
	serialDeviceCalibration->getCompositeDHParameterCalibration()->setLocked(!isLinkCalibrationEnabled);
	std::cout << "Initialized [ Base calibration: " << (!serialDeviceCalibration->getBaseCalibration()->isLocked() ? "Enabled" : "Disabled")
		<< " - End calibration: " << (!serialDeviceCalibration->getEndCalibration()->isLocked() ? "Enabled" : "Disabled") << " - Link calibration: "
		<< (!serialDeviceCalibration->getCompositeDHParameterCalibration()->isLocked() ? "Enabled" : "Disabled") << " ]." << std::endl;

	std::cout << "\tInitializing calibrator..";
	std::cout.flush();
	rwlibs::calibration::SerialDeviceCalibrator::Ptr serialDeviceCalibrator = rw::common::ownedPtr(
		new rwlibs::calibration::SerialDeviceCalibrator(serialDevice, referenceFrame, measurementFrame, serialDeviceCalibration));
	if (measurements.size() < serialDeviceCalibrator->getMinimumMeasurementCount()) {
		std::cout << "Not enough measurements." << std::endl;
		return -1;
	}
	serialDeviceCalibrator->setMeasurements(measurements);
	serialDeviceCalibrator->setWeightingEnabled(isWeightingEnabled);
	std::cout << "Initialized [ Min. measurements: " << serialDeviceCalibrator->getMinimumMeasurementCount() << " -  Weighting: "
		<< (serialDeviceCalibrator->isWeightingEnabled() ? "Enabled" : "Disabled") << " ]." << std::endl;

	// Run calibrator.
	std::cout << "Calibrating.. " << std::endl;
	std::cout.flush();
	try {
		serialDeviceCalibrator->calibrate(state);
		printSolverLog(serialDeviceCalibrator->getSolverLog());
		std::cout << "\tSucceeded." << std::endl;
	} catch (rw::common::Exception& exception) {
		printSolverLog(serialDeviceCalibrator->getSolverLog());
		std::cout << "\tFAILED: " << exception.getMessage() << std::endl;
	}

	// Save calibration.
	if (!calibrationFilePath.empty()) {
		std::cout << "Saving calibration [" << calibrationFilePath << "].. ";
		std::cout.flush();
		rwlibs::calibration::XmlCalibrationSaver::save(serialDeviceCalibration, calibrationFilePath);
		std::cout << "Saved." << std::endl;
	}

	// Print calibration summary.
	std::cout << "Calibration summary:" << std::endl;
	printCalibrationSummary(serialDeviceCalibration);

	// Print differences between model and measurements.
	std::cout << "Residual summary:" << std::endl;
	printMeasurements(measurements, serialDeviceCalibration);

	return 0;
}

int parseArguments(int argumentCount, char** arguments) {
	optionsDescription.add_options()("help", "Print help message")("workCellFile", boost::program_options::value<std::string>(&workCellFilePath)->required(),
		"Set the work cell file path")("measurementFile", boost::program_options::value<std::string>(&measurementFilePath)->required(), "Set the measurement file path")(
		"calibrationFile", boost::program_options::value<std::string>(&calibrationFilePath), "Set the calibration file path")("device",
		boost::program_options::value<std::string>(&deviceName), "Set the device name")("referenceFrame",
		boost::program_options::value<std::string>(&referenceFrameName), "Set the reference frame name")("measurementFrame",
		boost::program_options::value<std::string>(&measurementFrameName), "Set the measurement frame name")("weightingEnabled",
		boost::program_options::value<bool>(&isWeightingEnabled)->default_value(true),
		"Enable/disable measurement weighting")("baseCalibrationEnabled",
		boost::program_options::value<bool>(&isBaseCalibrationEnabled)->default_value(true), "Enable/disable calibration of base transformation")("endCalibrationEnabled",
		boost::program_options::value<bool>(&isEndCalibrationEnabled)->default_value(true),
		"Disable calibration of end transformation")("linkCalibrationEnabled",
		boost::program_options::value<bool>(&isLinkCalibrationEnabled)->default_value(true), "Enable/disable calibration of link transformations");

	positionalOptionsDescription.add("workCellFile", 1).add("measurementFile", 1).add("calibrationFile", 1);

	try {
		boost::program_options::store(
			boost::program_options::command_line_parser(argumentCount, arguments).options(optionsDescription).positional(positionalOptionsDescription).run(),
			variablesMap);

		if (variablesMap.count("help")) {
			printHelp();
			return 0;
		}

		boost::program_options::notify(variablesMap);
	} catch (boost::program_options::error& error) {
		std::cerr << "Error: " << error.what() << std::endl;
		std::cerr << std::endl;
		printHelp();
		return -1;
	}

	return 1;
}

void printHelp() {
	std::cerr << "Usage:" << std::endl;
	std::cerr << "  rw_calibration-tool [work cell file] [measurement file] [calibration file]" << std::endl;
	std::cerr << std::endl;
	std::cerr << optionsDescription << std::endl;
}

void printMeasurements(const std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr>& measurements, rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibration) {
	const unsigned int measurementCount = measurements.size();

	Eigen::VectorXd distances(measurementCount), angles(measurementCount);
	Eigen::VectorXd calibratedDistances(measurementCount), calibratedAngles(measurementCount);
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		serialDevice->setQ(measurements[measurementIndex]->getQ(), state);

		const rw::math::Transform3D<> tfmMeasurement = measurements[measurementIndex]->getTransform();
		const rw::math::Transform3D<> tfmModel = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state);
		serialDeviceCalibration->apply();
		const rw::math::Transform3D<> tfmCalibratedModel =
			rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state);
		serialDeviceCalibration->revert();
		const rw::math::Transform3D<> tfmError = rw::math::Transform3D<>(tfmModel.P() - tfmMeasurement.P(), tfmModel.R() * rw::math::inverse(tfmMeasurement.R()));
		const rw::math::Transform3D<> tfmCalibratedError = rw::math::Transform3D<>(tfmCalibratedModel.P() - tfmMeasurement.P(), tfmCalibratedModel.R() * rw::math::inverse(tfmMeasurement.R()));

		distances(measurementIndex) = tfmError.P().norm2(), calibratedDistances(measurementIndex) = tfmCalibratedError.P().norm2();
		angles(measurementIndex) = rw::math::EAA<>(tfmError.R()).angle(), calibratedAngles(measurementIndex) = rw::math::EAA<>(
			tfmCalibratedError.R()).angle();

		std::cout << "\tMeasurement " << measurementIndex + 1 << ": [ Uncalibrated: " << distances(measurementIndex) * 100.0 << " cm / "
			<< angles(measurementIndex) * rw::math::Rad2Deg << " \u00B0 - Calibrated: " << calibratedDistances(measurementIndex) * 100.0 << " cm / "
			<< calibratedAngles(measurementIndex) * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;
	}
	std::cout << "\tSummary - Uncalibrated: [ Avg: " << distances.mean() * 100.0 << " cm / " << angles.mean() * rw::math::Rad2Deg << " \u00B0 - Min: "
		<< distances.minCoeff() * 100.0 << " cm / " << angles.minCoeff() * rw::math::Rad2Deg << " \u00B0 - Max: " << distances.maxCoeff() * 100.0 << " cm / "
		<< angles.maxCoeff() * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;
	std::cout << "\tSummary - Calibrated: [ Avg: " << calibratedDistances.mean() * 100.0 << " cm / " << calibratedAngles.mean() * rw::math::Rad2Deg << " \u00B0 - Min: "
		<< calibratedDistances.minCoeff() * 100.0 << " cm / " << calibratedAngles.minCoeff() * rw::math::Rad2Deg << " \u00B0 - Max: "
		<< calibratedDistances.maxCoeff() * 100.0 << " cm / " << calibratedAngles.maxCoeff() * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;
}

void printSolverLog(rwlibs::calibration::NLLSSolverLog::Ptr solverLog) {
	std::vector<rwlibs::calibration::NLLSIterationLog> iterationLogs = solverLog->getIterationLogs();
	for (std::vector<rwlibs::calibration::NLLSIterationLog>::const_iterator it = iterationLogs.begin(); it != iterationLogs.end(); it++) {
		rwlibs::calibration::NLLSIterationLog iterationLog = *it;
		std::cout << "\tIteration " << iterationLog.getIterationNumber() << ": Jacobian [ Singular: " << (iterationLog.isSingular() ? "Yes" : "No")
			<< " - Condition: " << iterationLog.getConditionNumber() << " ] - ||Residuals||: " << iterationLog.getResidualNorm() << " - ||Step||: "
			<< iterationLog.getStepNorm() << std::endl;
	}
}

void printCalibrationSummary(rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibration) {
	rwlibs::calibration::FixedFrameCalibration::Ptr baseCalibration = serialDeviceCalibration->getBaseCalibration();
	std::cout << "\tBase calibration of [ " << baseCalibration->getFrame()->getName() << " ]: [ Translation: "
		<< baseCalibration->getCorrection().P().norm2() * 100.0 << " cm - Rotation: "
		<< rw::math::EAA<>(baseCalibration->getCorrection().R()).angle() * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;

	rwlibs::calibration::FixedFrameCalibration::Ptr endCalibration = serialDeviceCalibration->getEndCalibration();
	std::cout << "\tEnd calibration of [ " << endCalibration->getFrame()->getName() << " ]: [ Translation: "
		<< endCalibration->getCorrection().P().norm2() * 100.0 << " cm - Rotation: "
		<< rw::math::EAA<>(endCalibration->getCorrection().R()).angle() * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;

	rwlibs::calibration::CompositeCalibration<rwlibs::calibration::DHParameterCalibration>::Ptr dhParameterCalibration =
		serialDeviceCalibration->getCompositeDHParameterCalibration();
	for (unsigned int calibrationIndex = 0; calibrationIndex < dhParameterCalibration->getCalibrations().size(); calibrationIndex++) {
		rwlibs::calibration::DHParameterCalibration::Ptr dhCalibration = dhParameterCalibration->getCalibrations()[calibrationIndex];
		Eigen::Vector4d correction = dhCalibration->getCorrection();
		std::cout << "\tDH calibration of [ " << dhCalibration->getJoint()->getName() << " ]: [ a: "
			<< correction(rwlibs::calibration::DHParameterCalibration::PARAMETER_A) * 100.0 << " cm - b/d: "
			<< correction(rwlibs::calibration::DHParameterCalibration::PARAMETER_B_D) * 100.0 << " cm - alpha: "
			<< correction(rwlibs::calibration::DHParameterCalibration::PARAMETER_ALPHA) * rw::math::Rad2Deg << " \u00B0 - beta/theta: "
			<< correction(rwlibs::calibration::DHParameterCalibration::PARAMETER_BETA_THETA) * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;
	}
}
