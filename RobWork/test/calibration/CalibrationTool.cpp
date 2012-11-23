#include "MultivariateNormalDistribution.hpp"
#include <rw/common.hpp>
#include <rw/loaders.hpp>
#include <rwlibs/calibration.hpp>

bool isPrintingHelp;
std::string workCellFilePath;
std::string deviceName;
std::string referenceFrameName;
std::string measurementFrameName;
std::string measurementFilePath;
std::string calibrationFilePath;
bool isWeightingMeasurements;
bool isBaseCalibrationEnabled;
bool isEndCalibrationEnabled;
bool isLinkCalibrationEnabled;

void parseArguments(int argumentCount, char** arguments, boost::program_options::options_description& optionsDescription);
void printHelp(const boost::program_options::options_description& optionsDescription);
void printSolverLog(const rwlibs::calibration::NLLSSolverLog& solverLog);
void printCalibrationSummary(rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibration);
void printMeasurements(const std::vector<rwlibs::calibration::SerialDevicePoseMeasurement>& measurements, rw::models::WorkCell::Ptr workCell, rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibration);

int main(int argumentCount, char** arguments) {
	std::cout << "Parsing arguments.. ";
	std::cout.flush();
	boost::program_options::options_description optionsDescription("Options");
	try {
		parseArguments(argumentCount, arguments, optionsDescription);
		std::cout << "Parsed." << std::endl;
	} catch(rw::common::Exception& exception) {
		std::cout << "FAILED: " << exception.getMessage() << std::endl;
		printHelp(optionsDescription);
		return -1;
	}

	if (isPrintingHelp) {
		printHelp(optionsDescription);
		return 0;
	}

	// Load workcell.
	std::cout << "Loading work cell [ " << workCellFilePath << " ].. ";
	std::cout.flush();
	rw::models::WorkCell::Ptr workCell = rw::loaders::XMLRWLoader::load(workCellFilePath);
	if (workCell.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Loaded [ " << workCell->getName() << " ]." << std::endl;

	// Find device and cast to serial device.
	std::cout << "Finding device [ " << deviceName << " ].. ";
	std::cout.flush();
	rw::models::Device::Ptr device = deviceName.empty() ? workCell->getDevices().front() : workCell->findDevice(deviceName);
	if (device.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Found [ " << device->getName() << " ]." << std::endl;
	rw::models::SerialDevice::Ptr serialDevice = device.cast<rw::models::SerialDevice>();

	// Find reference frame.
	std::cout << "Finding reference frame [ " << referenceFrameName << " ].. ";
	std::cout.flush();
	rw::kinematics::Frame::Ptr referenceFrame = referenceFrameName.empty() ? workCell->findFrame("WORLD") : workCell->findFrame(referenceFrameName);
	if (referenceFrame.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Found [ " << referenceFrame->getName() << " ]." << std::endl;

	// Find measurement frame.
	std::cout << "Finding measurement frame [ " << measurementFrameName << " ].. ";
	std::cout.flush();
	rw::kinematics::Frame::Ptr measurementFrame = measurementFrameName.empty() ? device->getEnd() : workCell->findFrame(measurementFrameName);
	if (measurementFrame.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Found [ " << measurementFrame->getName() << " ]." << std::endl;

	// Load robot pose measurements from file.
	std::cout << "Loading measurements [ " << measurementFilePath << " ].. ";
	std::cout.flush();
	std::vector<rwlibs::calibration::SerialDevicePoseMeasurement> measurements = rwlibs::calibration::XmlMeasurementFile::load(measurementFilePath);
	std::cout << "Loaded " << measurements.size() << " measurements." << std::endl;

	// Disable existing calibration if one exist.
	std::cout << "Finding existing calibration.. ";
	std::cout.flush();
	rwlibs::calibration::SerialDeviceCalibration::Ptr calibrationExisting = rwlibs::calibration::SerialDeviceCalibration::get(serialDevice);
	if (calibrationExisting.isNull()) {
		std::cout << "Not found." << std::endl;
	} else {
		std::cout << "Found." << std::endl;

		std::cout << "Disabling existing calibration.. ";
		std::cout.flush();
		calibrationExisting->revert();
		std::cout << "Disabled." << std::endl;
	}

	// Initialize calibration, jacobian and calibrator.
	std::cout << "Initializing calibration.. ";
	std::cout.flush();
	rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibration = rw::common::ownedPtr(new rwlibs::calibration::SerialDeviceCalibration(serialDevice));
	std::cout << "Initialized." << std::endl;
	
	std::cout << "Initializing jacobian [ Base: " << (isBaseCalibrationEnabled ? "Enabled" : "Disabled") << " - End: " << (isEndCalibrationEnabled ? "Enabled" : "Disabled") << " - Link: " << (isLinkCalibrationEnabled ? "Enabled" : "Disabled") << " ].. ";
	std::cout.flush();
	rwlibs::calibration::SerialDeviceJacobian::Ptr serialDeviceJacobian = rw::common::ownedPtr(new rwlibs::calibration::SerialDeviceJacobian(serialDeviceCalibration));
	serialDeviceJacobian->getBaseJacobian()->setEnabled(isBaseCalibrationEnabled);
	serialDeviceJacobian->getEndJacobian()->setEnabled(isEndCalibrationEnabled);
	serialDeviceJacobian->getCompositeDHParameterJacobian()->setEnabled(isLinkCalibrationEnabled);
	std::cout << "Initialized." << std::endl;

	std::cout << "Initializing calibrator [ Weighting: " << (isWeightingMeasurements ? "Enabled" : "Disabled") << " ].. ";
	std::cout.flush();
	rwlibs::calibration::SerialDeviceCalibrator::Ptr serialDeviceCalibrator = rw::common::ownedPtr(
		new rwlibs::calibration::SerialDeviceCalibrator(serialDevice, referenceFrame, measurementFrame, serialDeviceCalibration, serialDeviceJacobian));
	serialDeviceCalibrator->setMeasurements(measurements);
	serialDeviceCalibrator->setWeightingMeasurements(isWeightingMeasurements);
	std::cout << "Initialized." << std::endl;

	std::cout << "Calibrating.. ";
	std::cout.flush();
	try {
		// Run calibrator.
		serialDeviceCalibrator->calibrate(workCell->getDefaultState());
		std::cout << "Calibrated." << std::endl;

		// Save calibration.
		if (!calibrationFilePath.empty()) {
			std::cout << "Saving calibration [" << calibrationFilePath << "].. ";
			std::cout.flush();
			rwlibs::calibration::XmlCalibrationSaver::save(serialDeviceCalibration, calibrationFilePath);
			std::cout << "Saved." << std::endl;
		}

		std::cout << "Solver summary:" << std::endl;
		printSolverLog(serialDeviceCalibrator->getSolverLog());
	} catch (rw::common::Exception& exception) {
		std::cout << "FAILED: " << exception.getMessage() << std::endl;

		std::cout << "Solver log:" << std::endl;
		printSolverLog(serialDeviceCalibrator->getSolverLog());
	}

	// Print calibration summary.
	std::cout << "Calibration summary:" << std::endl;
	printCalibrationSummary(serialDeviceCalibration);

	// Print differences between model and measurements.
	std::cout << "Residual summary:" << std::endl;
	printMeasurements(measurements, workCell, serialDevice, referenceFrame, measurementFrame, serialDeviceCalibration);

	return 0;
}

void parseArguments(int argumentCount, char** arguments, boost::program_options::options_description& optionsDescription) {
	optionsDescription.add_options()("help", boost::program_options::value<bool>(&isPrintingHelp)->default_value(false), "Print help message")("workCellFile", boost::program_options::value<std::string>(&workCellFilePath)->required(),
		"Set the work cell file path")("measurementFile", boost::program_options::value<std::string>(&measurementFilePath)->required(), "Set the measurement file path")(
		"calibrationFile", boost::program_options::value<std::string>(&calibrationFilePath), "Set the calibration file path")("device",
		boost::program_options::value<std::string>(&deviceName), "Set the device name")("referenceFrame",
		boost::program_options::value<std::string>(&referenceFrameName), "Set the reference frame name")("measurementFrame",
		boost::program_options::value<std::string>(&measurementFrameName), "Set the measurement frame name")("weightingMeasurements",
		boost::program_options::value<bool>(&isWeightingMeasurements)->default_value(true),
		"Enable/disable weighting of measurements")("enableBaseCalibration",
		boost::program_options::value<bool>(&isBaseCalibrationEnabled)->default_value(true), "Enable/disable calibration of base transformation")("enableEndCalibration",
		boost::program_options::value<bool>(&isEndCalibrationEnabled)->default_value(true),
		"Disable calibration of end transformation")("enableLinkCalibration",
		boost::program_options::value<bool>(&isLinkCalibrationEnabled)->default_value(true), "Enable/disable calibration of link transformations");

	boost::program_options::positional_options_description positionalOptionsDescription;
	positionalOptionsDescription.add("workCellFile", 1).add("measurementFile", 1).add("calibrationFile", 1);

	try {
		boost::program_options::variables_map variablesMap;
		boost::program_options::store(
			boost::program_options::command_line_parser(argumentCount, arguments).options(optionsDescription).positional(positionalOptionsDescription).run(),
			variablesMap);
		boost::program_options::notify(variablesMap);
	} catch (boost::program_options::error& error) {
		RW_THROW(error.what());
	}
}

void printHelp(const boost::program_options::options_description& optionsDescription) {
	std::cerr << "Usage:" << std::endl;
	std::cerr << "  rw_calibration-tool [work cell file] [measurement file] [calibration file]" << std::endl;
	std::cerr << std::endl;
	std::cerr << optionsDescription << std::endl;
}

void printSolverLog(const rwlibs::calibration::NLLSSolverLog& solverLog) {
	std::vector<rwlibs::calibration::NLLSIterationLog> iterationLogs = solverLog.getIterationLogs();
	for (std::vector<rwlibs::calibration::NLLSIterationLog>::const_iterator it = iterationLogs.begin(); it != iterationLogs.end(); it++) {
		rwlibs::calibration::NLLSIterationLog iterationLog = (*it);
		std::cout << "\tIteration " << iterationLog.getIterationNumber() << ": Jacobian [ Singular: " << (iterationLog.isSingular() ? "Yes" : "No")
			<< " - Condition: " << iterationLog.getConditionNumber() << " ] - ||Residuals||: " << iterationLog.getResidualNorm() << " - ||Step||: "
			<< iterationLog.getStepNorm() << std::endl;
	}
}

void printCalibrationSummary(rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibration) {
	rwlibs::calibration::FixedFrameCalibration::Ptr baseCalibration = serialDeviceCalibration->getBaseCalibration();
	rw::math::Transform3D<> baseCalibrationCorrectionTransform = baseCalibration->getCorrectionTransform();
	std::cout << "\tBase calibration of [ " << baseCalibration->getFrame()->getName() << " ]: [ Translation: "
		<< baseCalibrationCorrectionTransform.P().norm2() * 100.0 << " cm - Rotation: "
		<< rw::math::EAA<>(baseCalibrationCorrectionTransform.R()).angle() * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;

	rwlibs::calibration::FixedFrameCalibration::Ptr endCalibration = serialDeviceCalibration->getEndCalibration();
	rw::math::Transform3D<> endCalibrationCorrectionTransform = endCalibration->getCorrectionTransform();
	std::cout << "\tEnd calibration of [ " << endCalibration->getFrame()->getName() << " ]: [ Translation: "
		<< endCalibrationCorrectionTransform.P().norm2() * 100.0 << " cm - Rotation: "
		<< rw::math::EAA<>(endCalibrationCorrectionTransform.R()).angle() * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;

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

void printMeasurements(const std::vector<rwlibs::calibration::SerialDevicePoseMeasurement>& measurements, rw::models::WorkCell::Ptr workCell, rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibration) {
	const unsigned int measurementCount = measurements.size();

	Eigen::VectorXd distances(measurementCount), angles(measurementCount);
	Eigen::VectorXd calibratedDistances(measurementCount), calibratedAngles(measurementCount);
	rw::kinematics::State state = workCell->getDefaultState();
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		serialDevice->setQ(measurements[measurementIndex].getQ(), state);

		const rw::math::Transform3D<> tfmMeasurement = measurements[measurementIndex].getTransform();
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
