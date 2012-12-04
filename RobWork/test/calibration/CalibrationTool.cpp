#include "MultivariateNormalDistribution.hpp"
#include <rw/common.hpp>
#include <rw/loaders.hpp>
#include <rwlibs/calibration.hpp>

using namespace rwlibs::calibration;

bool isPrintingHelp;
bool isPrintingDetails;
std::string workCellFilePath;
std::string deviceName;
std::string referenceFrameName;
std::string measurementFrameName;
std::string measurementFilePath;
std::string calibrationFilePath;
bool isWeighting;
bool isBaseCalibrationEnabled;
bool isEndCalibrationEnabled;
bool isLinkCalibrationEnabled;
int validationMeasurementCount;

void parseArguments(int argumentCount, char** arguments, boost::program_options::options_description& optionsDescription);
void printHelp(const boost::program_options::options_description& optionsDescription);
void printSolverSummary(NLLSSolver::Ptr solver);
void printCalibrationSummary(const SerialDeviceCalibration::Ptr serialDeviceCalibration);
void printFixedFrameCalibrationSummary(const FixedFrameCalibration::Ptr calibration);
void printDHCalibrationSummary(const DHParameterCalibration::Ptr calibration);
void printMeasurements(const std::vector<SerialDevicePoseMeasurement>& measurements, rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, SerialDeviceCalibration::Ptr serialDeviceCalibration);
void printMeasurementSummary(const std::vector<SerialDevicePoseMeasurement>& measurements, rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, SerialDeviceCalibration::Ptr serialDeviceCalibration);

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
	const rw::kinematics::State defaultState = workCell->getDefaultState();
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
	std::vector<SerialDevicePoseMeasurement> allMeasurements = XmlMeasurementFile::load(measurementFilePath);
	const int allMeasurementCount = allMeasurements.size();
	std::vector<SerialDevicePoseMeasurement> measurements, validationMeasurements;
	if (validationMeasurementCount > 0) {
		for (int measurementIndex = 0; measurementIndex < allMeasurementCount; measurementIndex ++) {
			if (measurementIndex % 2 == 0 && validationMeasurements.size() < validationMeasurementCount)
				validationMeasurements.push_back(allMeasurements[measurementIndex]);
			else
				measurements.push_back(allMeasurements[measurementIndex]);
		}
	}
	else
		measurements = allMeasurements;
	std::cout << "Loaded " << allMeasurementCount << " measurements (" << validationMeasurementCount <<  " reserved for validation)." << std::endl;
	RW_ASSERT(measurements.size() == (allMeasurementCount - validationMeasurementCount));
	RW_ASSERT(validationMeasurements.size() == validationMeasurementCount);

	// Disable existing calibration if one exist.
	std::cout << "Finding existing calibration.. ";
	std::cout.flush();
	SerialDeviceCalibration::Ptr calibrationExisting = SerialDeviceCalibration::get(serialDevice);
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
	SerialDeviceCalibration::Ptr serialDeviceCalibration = rw::common::ownedPtr(new SerialDeviceCalibration(serialDevice));
	std::cout << "Initialized." << std::endl;

	std::cout << "Initializing jacobian.. ";
	std::cout.flush();
	SerialDeviceJacobian::Ptr serialDeviceJacobian = rw::common::ownedPtr(new SerialDeviceJacobian(serialDeviceCalibration));
	std::cout << "Initialized." << std::endl;

	std::cout << "Initializing calibrator [ Weighting: " << (isWeighting ? "Enabled" : "Disabled") << " ].. ";
	std::cout.flush();
	SerialDeviceCalibrator::Ptr serialDeviceCalibrator = rw::common::ownedPtr(
		new SerialDeviceCalibrator(serialDevice, referenceFrame, measurementFrame, serialDeviceCalibration, serialDeviceJacobian));
	serialDeviceCalibrator->setMeasurements(measurements);
	serialDeviceCalibrator->setWeighting(isWeighting);
	std::cout << "Initialized." << std::endl;

	try {
		// Run calibrator.
		std::cout << "Calibrating [ Base: " << (isBaseCalibrationEnabled ? "Enabled" : "Disabled") << " - End: " << (isEndCalibrationEnabled ? "Enabled" : "Disabled") << " - Link: " << (isLinkCalibrationEnabled ? "Enabled" : "Disabled") << " ].. ";
		std::cout.flush();
		serialDeviceCalibration->getBaseCalibration()->setEnabled(isBaseCalibrationEnabled);
		serialDeviceCalibration->getEndCalibration()->setEnabled(isEndCalibrationEnabled);
		serialDeviceCalibration->getInternalLinkCalibration()->setEnabled(isLinkCalibrationEnabled);
		serialDeviceCalibrator->calibrate(defaultState);
		const int iterationCount = serialDeviceCalibrator->getSolver()->getIterationCount();
		std::cout << "Calibrated (in " << iterationCount << " iterations)." << std::endl;

		// Save calibration.
		if (!calibrationFilePath.empty()) {
			std::cout << "Saving calibration [" << calibrationFilePath << "].. ";
			std::cout.flush();
			XmlCalibrationSaver::save(serialDeviceCalibration, calibrationFilePath);
			std::cout << "Saved." << std::endl;
		}

		if (isPrintingDetails) {
			std::cout << "Solver summary:" << std::endl;
			printSolverSummary(serialDeviceCalibrator->getSolver());
		}
	} catch (rw::common::Exception& exception) {
		std::cout << "FAILED: " << exception.getMessage() << std::endl;

		std::cout << "Solver log:" << std::endl;
		printSolverSummary(serialDeviceCalibrator->getSolver());
	}

	// Print calibration summary.
	std::cout << "Calibration summary:" << std::endl;
	printCalibrationSummary(serialDeviceCalibration);

	// Print differences between model and measurements.
	std::cout << "Residual summary:" << std::endl;
	if (isPrintingDetails)
		printMeasurements(measurements, serialDevice, referenceFrame, measurementFrame, workCell->getDefaultState(), serialDeviceCalibration);
	printMeasurementSummary(measurements, serialDevice, referenceFrame, measurementFrame, workCell->getDefaultState(), serialDeviceCalibration);

	// Print differences between model and validation measurements.
	if (validationMeasurementCount > 0) {
		std::cout << "Residual summary (validation):" << std::endl;
		if (isPrintingDetails)
			printMeasurements(validationMeasurements, serialDevice, referenceFrame, measurementFrame, workCell->getDefaultState(), serialDeviceCalibration);
		printMeasurementSummary(validationMeasurements, serialDevice, referenceFrame, measurementFrame, workCell->getDefaultState(), serialDeviceCalibration);
	}

	return 0;
}

void parseArguments(int argumentCount, char** arguments, boost::program_options::options_description& optionsDescription) {
	optionsDescription.add_options()
		("help", boost::program_options::value<bool>(&isPrintingHelp)->zero_tokens(), "Print help message")
		("details", boost::program_options::value<bool>(&isPrintingDetails)->zero_tokens(), "Print details")
		("workCellFile", boost::program_options::value<std::string>(&workCellFilePath)->required(), "Set the work cell file path")
		("measurementFile", boost::program_options::value<std::string>(&measurementFilePath)->required(), "Set the measurement file path")
		("calibrationFile", boost::program_options::value<std::string>(&calibrationFilePath), "Set the calibration file path")
		("device", boost::program_options::value<std::string>(&deviceName), "Set the device name")
		("referenceFrame", boost::program_options::value<std::string>(&referenceFrameName), "Set the reference frame name")
		("measurementFrame", boost::program_options::value<std::string>(&measurementFrameName), "Set the measurement frame name")
		("weighting", boost::program_options::value<bool>(&isWeighting)->default_value(true), "Enable/disable weighting of measurements")
		("enableBaseCalibration", boost::program_options::value<bool>(&isBaseCalibrationEnabled)->default_value(true), "Enable/disable calibration of base transformation")
		("enableEndCalibration", boost::program_options::value<bool>(&isEndCalibrationEnabled)->default_value(true), "Disable calibration of end transformation")
		("enableLinkCalibration", boost::program_options::value<bool>(&isLinkCalibrationEnabled)->default_value(true), "Enable/disable calibration of link transformations")
		("validationMeasurements", boost::program_options::value<int>(&validationMeasurementCount)->default_value(0), "Number of measurements to reserve for validation");

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

void printSolverSummary(NLLSSolver::Ptr solver) {
	std::vector<NLLSIterationLog> iterationLogs = solver->getIterationLogs();
	for (std::vector<NLLSIterationLog>::const_iterator it = iterationLogs.begin(); it != iterationLogs.end(); it++) {
		NLLSIterationLog iterationLog = (*it);
		std::cout << "\tIteration " << iterationLog.getIterationNumber() << ": Jacobian [ Singular: " << (iterationLog.isSingular() ? "Yes" : "No")
			<< " - Condition: " << iterationLog.getConditionNumber() << " ] - ||Residuals||: " << iterationLog.getResidualNorm() << " - ||Step||: "
			<< iterationLog.getStepNorm() << std::endl;
	}
}

void printCalibrationSummary(const SerialDeviceCalibration::Ptr serialDeviceCalibration) {
	FixedFrameCalibration::Ptr baseCalibration = serialDeviceCalibration->getBaseCalibration();
	std::cout << "\tBase calibration of [ " << baseCalibration->getFrame()->getName() << " ]: ";
	printFixedFrameCalibrationSummary(baseCalibration);

	FixedFrameCalibration::Ptr endCalibration = serialDeviceCalibration->getEndCalibration();
	std::cout << "\tEnd calibration of [ " << endCalibration->getFrame()->getName() << " ]: ";
	printFixedFrameCalibrationSummary(endCalibration);

	CompositeCalibration<DHParameterCalibration>::Ptr dhParameterCalibration =
		serialDeviceCalibration->getInternalLinkCalibration();
	for (unsigned int calibrationIndex = 0; calibrationIndex < dhParameterCalibration->getCalibrations().size(); calibrationIndex++) {
		DHParameterCalibration::Ptr dhCalibration = dhParameterCalibration->getCalibrations()[calibrationIndex];
		std::cout << "\tDH calibration of [ " << dhCalibration->getJoint()->getName() << " ]: ";
		printDHCalibrationSummary(dhCalibration);
	}
}

void printFixedFrameCalibrationSummary(const FixedFrameCalibration::Ptr calibration) {
	std::cout << "[";
	const CalibrationParameterSet parameterSet = calibration->getParameterSet();
	if (parameterSet(FixedFrameCalibration::PARAMETER_X).isEnabled())
		std::cout << " x: " << parameterSet(FixedFrameCalibration::PARAMETER_X) * 1000.0 << " mm";
	if (parameterSet(FixedFrameCalibration::PARAMETER_Y).isEnabled())
		std::cout << " y: " << parameterSet(FixedFrameCalibration::PARAMETER_Y) * 1000.0 << " mm";
	if (parameterSet(FixedFrameCalibration::PARAMETER_Z).isEnabled())
		std::cout << " z: " << parameterSet(FixedFrameCalibration::PARAMETER_Z) * 1000.0 << " mm";
	if (parameterSet(FixedFrameCalibration::PARAMETER_ROLL).isEnabled())
		std::cout << " roll: " << parameterSet(FixedFrameCalibration::PARAMETER_ROLL) * rw::math::Rad2Deg << " \u00B0";
	if (parameterSet(FixedFrameCalibration::PARAMETER_PITCH).isEnabled())
		std::cout << " pitch: " << parameterSet(FixedFrameCalibration::PARAMETER_PITCH) * rw::math::Rad2Deg << " \u00B0";
	if (parameterSet(FixedFrameCalibration::PARAMETER_YAW).isEnabled())
		std::cout << " yaw: " << parameterSet(FixedFrameCalibration::PARAMETER_YAW) * rw::math::Rad2Deg << " \u00B0";
	std::cout << " ]";

	std::cout << " [";
	const rw::math::Transform3D<> correctionTransform = calibration->getCorrectionTransform();
	std::cout << " Translation: " << correctionTransform.P().norm2() * 1000.0 << " mm";
	std::cout << " Rotation: " << rw::math::EAA<>(correctionTransform.R()).angle() * rw::math::Rad2Deg << " \u00B0";
	std::cout << " ]" << std::endl;
}

void printDHCalibrationSummary(const DHParameterCalibration::Ptr calibration) {
	std::cout << "[";
	CalibrationParameterSet parameterSet = calibration->getParameterSet();
	if (parameterSet(DHParameterCalibration::PARAMETER_A).isEnabled())
		std::cout << " a: " << parameterSet(DHParameterCalibration::PARAMETER_A) * 1000.0 << " mm";
	if (parameterSet(DHParameterCalibration::PARAMETER_B).isEnabled())
		std::cout << " b: " << parameterSet(DHParameterCalibration::PARAMETER_B) * 1000.0 << " mm";
	if (parameterSet(DHParameterCalibration::PARAMETER_D).isEnabled())
		std::cout << " d: " << parameterSet(DHParameterCalibration::PARAMETER_D) * 1000.0 << " mm";
	if (parameterSet(DHParameterCalibration::PARAMETER_ALPHA).isEnabled())
		std::cout << " alpha: "	<< parameterSet(DHParameterCalibration::PARAMETER_ALPHA) * rw::math::Rad2Deg << " \u00B0";
	if (parameterSet(DHParameterCalibration::PARAMETER_BETA).isEnabled())
		std::cout << " beta: " << parameterSet(DHParameterCalibration::PARAMETER_BETA) * rw::math::Rad2Deg << " \u00B0";
	if (parameterSet(DHParameterCalibration::PARAMETER_THETA).isEnabled())
		std::cout << " theta: " << parameterSet(DHParameterCalibration::PARAMETER_THETA) * rw::math::Rad2Deg << " \u00B0";
	std::cout << " ]" << std::endl;
}

void printMeasurements(const std::vector<SerialDevicePoseMeasurement>& measurements, rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, SerialDeviceCalibration::Ptr serialDeviceCalibration) {
	const unsigned int measurementCount = measurements.size();

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

		double distance = tfmError.P().norm2(), calibratedDistance = tfmCalibratedError.P().norm2();
		double angle = rw::math::EAA<>(tfmError.R()).angle(), calibratedAngle = rw::math::EAA<>(tfmCalibratedError.R()).angle();

		std::cout << "\tMeasurement " << measurementIndex + 1 << ": [ Uncalibrated: " << distance * 1000.0 << " mm / "
			<< angle * rw::math::Rad2Deg << " \u00B0 - Calibrated: " << calibratedDistance * 1000.0 << " mm / "
			<< calibratedAngle * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;
	}
}

void printMeasurementSummary(const std::vector<SerialDevicePoseMeasurement>& measurements, rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, SerialDeviceCalibration::Ptr serialDeviceCalibration) {
	const unsigned int measurementCount = measurements.size();

	Eigen::VectorXd distances(measurementCount), angles(measurementCount);
	Eigen::VectorXd calibratedDistances(measurementCount), calibratedAngles(measurementCount);
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
		angles(measurementIndex) = rw::math::EAA<>(tfmError.R()).angle(), calibratedAngles(measurementIndex) = rw::math::EAA<>(tfmCalibratedError.R()).angle();
	}
	std::cout << "\tSummary - Uncalibrated: [ Avg: " << distances.mean() * 1000.0 << " mm / " << angles.mean() * rw::math::Rad2Deg << " \u00B0 - Min: "
		<< distances.minCoeff() * 1000.0 << " mm / " << angles.minCoeff() * rw::math::Rad2Deg << " \u00B0 - Max: " << distances.maxCoeff() * 1000.0 << " mm / "
		<< angles.maxCoeff() * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;
	std::cout << "\tSummary - Calibrated: [ Avg: " << calibratedDistances.mean() * 1000.0 << " mm / " << calibratedAngles.mean() * rw::math::Rad2Deg << " \u00B0 - Min: "
		<< calibratedDistances.minCoeff() * 1000.0 << " mm / " << calibratedAngles.minCoeff() * rw::math::Rad2Deg << " \u00B0 - Max: "
		<< calibratedDistances.maxCoeff() * 1000.0 << " mm / " << calibratedAngles.maxCoeff() * rw::math::Rad2Deg << " \u00B0 ]" << std::endl;
}
