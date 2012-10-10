#include <rw/common.hpp>
#include <rw/loaders.hpp>
#include <rwlibs/calibration.hpp>

std::string workCellFilePath;
std::string deviceName;
std::string referenceFrameName;
std::string measurementFrameName;
std::string measurementFilePath;
std::string calibrationFilePath;
bool weighting;
bool calibrateBase;
bool calibrateEnd;
bool calibrateDH;

rw::models::WorkCell::Ptr workCell;
rw::kinematics::State state;
rw::models::Device::Ptr device;
rw::models::SerialDevice::Ptr serialDevice;
rw::kinematics::Frame::Ptr referenceFrame;
rw::kinematics::Frame::Ptr measurementFrame;
std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> measurements;

rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibration;
rwlibs::calibration::SerialDeviceCalibrator::Ptr serialDeviceCalibrator;

void printMeasurementSummary();
void printSolverLog();
void printCalibrationSummary();

int main(int argumentCount, char** arguments) {
	std::cout << "Initializing calibration tool:" << std::endl;

	boost::program_options::options_description optionsDescription("Options");
	optionsDescription.add_options()("workCellFile", boost::program_options::value<std::string>(&workCellFilePath), "set the work cell file path")("device",
			boost::program_options::value<std::string>(&deviceName), "set the serial device name")("referenceFrame",
			boost::program_options::value<std::string>(&referenceFrameName), "set the reference frame")("measurementFrame",
			boost::program_options::value<std::string>(&measurementFrameName), "set the measurement frame")("measurementFile",
			boost::program_options::value<std::string>(&measurementFilePath), "set the measurement file")("calibrationFile",
			boost::program_options::value<std::string>(&calibrationFilePath), "set the calibration file")
			("weighting", boost::program_options::value<bool>(&weighting)->default_value(true), "enable weighting")
			("calibrateBase", boost::program_options::value<bool>(&calibrateBase)->default_value(true), "calibrate base")
			("calibrateEnd", boost::program_options::value<bool>(&calibrateEnd)->default_value(true), "calibrate end")
			("calibrateDH", boost::program_options::value<bool>(&calibrateDH)->default_value(true), "calibrate DH");

	boost::program_options::positional_options_description positionalOptionsDescription;
	positionalOptionsDescription.add("workCellFile", 1).add("measurementFile", 1).add("calibrationFile", 1);

	boost::program_options::variables_map variablesMap;
	boost::program_options::store(
			boost::program_options::command_line_parser(argumentCount, arguments).options(optionsDescription).positional(positionalOptionsDescription).run(),
			variablesMap);
	boost::program_options::notify(variablesMap);

	// Load workcell.
	std::cout << "\tLoading work cell..";
	std::cout.flush();
	workCell = rw::loaders::XMLRWLoader::load(workCellFilePath);
	if (workCell.isNull()) {
		std::cout << " FAILED." << std::endl;
		return -1;
	}
	std::cout << " Loaded [ " << workCell->getName() << " ]." << std::endl;
	state = workCell->getDefaultState();

	// Find device and frames.
	std::cout << "\tFinding device [ " << deviceName << " ]..";
	std::cout.flush();
	device = deviceName.empty() ? workCell->getDevices().front() : workCell->findDevice(deviceName);
	if (device.isNull()) {
		std::cout << " FAILED." << std::endl;
		return -1;
	}
	std::cout << " Found [ " << device->getName() << " ]." << std::endl;
	serialDevice = device.cast<rw::models::SerialDevice>();

	std::cout << "\tFinding reference frame [ " << referenceFrameName << " ]..";
	std::cout.flush();
	referenceFrame = referenceFrameName.empty() ? workCell->findFrame("WORLD") : workCell->findFrame(referenceFrameName);
	if (referenceFrame.isNull()) {
		std::cout << " FAILED." << std::endl;
		return -1;
	}
	std::cout << " Found [ " << referenceFrame->getName() << " ]." << std::endl;

	std::cout << "\tFinding measurement frame [ " << measurementFrameName << " ]..";
	std::cout.flush();
	measurementFrame = measurementFrameName.empty() ? device->getEnd() : workCell->findFrame(measurementFrameName);
	if (measurementFrame.isNull()) {
		std::cout << " FAILED." << std::endl;
		return -1;
	}
	std::cout << " Found [ " << measurementFrame->getName() << " ]." << std::endl;

	// Load robot pose measurements from file.
	std::cout << "\tLoading measurements [ " << measurementFilePath << " ]..";
	std::cout.flush();
	measurements = rwlibs::calibration::XmlMeasurementFile::load(measurementFilePath);
	std::cout << " Loaded " << measurements.size() << " measurements." << std::endl;

	// Disable existing calibration if one exist.
	std::cout << "\tFinding existing calibration..";
	std::cout.flush();
	rwlibs::calibration::SerialDeviceCalibration::Ptr calibrationExisting = rwlibs::calibration::SerialDeviceCalibration::get(serialDevice);
	if (calibrationExisting.isNull()) {
		std::cout << " Not found." << std::endl;
	} else {
		std::cout << " Found." << std::endl;

		std::cout << "\t\tDisabling existing calibration..";
		std::cout.flush();
		calibrationExisting->revert();
		std::cout << " Disabled." << std::endl;
	}

	// Initialize calibration, jacobian and calibrator.
	std::cout << "\tInitializing calibration..";
	std::cout.flush();
	serialDeviceCalibration = rw::common::ownedPtr(new rwlibs::calibration::SerialDeviceCalibration(serialDevice));
	serialDeviceCalibration->getBaseCalibration()->setLocked(!calibrateBase);
	serialDeviceCalibration->getEndCalibration()->setLocked(!calibrateEnd);
	serialDeviceCalibration->getCompositeDHParameterCalibration()->setLocked(!calibrateDH);
	std::cout << " Initialized [ Base calibration: " << (!serialDeviceCalibration->getBaseCalibration()->isLocked() ? "Enabled" : "Disabled")
			<< " - End calibration: " << (!serialDeviceCalibration->getEndCalibration()->isLocked() ? "Enabled" : "Disabled") << " - DH calibration: "
			<< (!serialDeviceCalibration->getCompositeDHParameterCalibration()->isLocked() ? "Enabled" : "Disabled") << " ]." << std::endl;

	std::cout << "\tInitializing calibrator..";
	std::cout.flush();
	serialDeviceCalibrator = rw::common::ownedPtr(
			new rwlibs::calibration::SerialDeviceCalibrator(serialDevice, state, referenceFrame, measurementFrame, serialDeviceCalibration));
	if (measurements.size() < serialDeviceCalibrator->getMinimumMeasurementCount()) {
		std::cout << "Not enough measurements." << std::endl;
		return -1;
	}
	serialDeviceCalibrator->setMeasurements(measurements);
	serialDeviceCalibrator->setWeightingEnabled(weighting);
	std::cout << " Initialized [ Min. measurements: " << serialDeviceCalibrator->getMinimumMeasurementCount() << " -  Weighting: " << (serialDeviceCalibrator->isWeightingEnabled() ? "Enabled" : "Disabled") << " ]." << std::endl;

	// Print differences between model and measurements before calibration.
	std::cout << "Measurement summary before calibration:" << std::endl;
	printMeasurementSummary();

	// Run calibrator.
	std::cout << "Calibrating.." << std::endl;
	std::cout.flush();
	try {
		serialDeviceCalibrator->calibrate();
		printSolverLog();
		std::cout << "\tSucceeded." << std::endl;
	} catch (rw::common::Exception& exception) {
		printSolverLog();
		std::cout << "\tFAILED." << std::endl;
		throw exception;
	}

	// Print calibration summary.
	std::cout << "Calibration summary:" << std::endl;
	printCalibrationSummary();

	// Apply calibration.
	std::cout << "Applying calibration..";
	std::cout.flush();
	serialDeviceCalibration->apply();
	std::cout << " Applied." << std::endl;

	// Print differences between model and measurements after calibration.
	std::cout << "Measurement summary after calibration:" << std::endl;
	printMeasurementSummary();

	// Save and load calibration.
	std::cout << "Saving calibration [" << calibrationFilePath << "]..";
	std::cout.flush();
	rwlibs::calibration::XmlCalibrationSaver::save(serialDeviceCalibration, calibrationFilePath);
	std::cout << " Saved." << std::endl;

	return 0;
}

void printMeasurementSummary() {
	const unsigned int measurementCount = measurements.size();

	double minPositionError = DBL_MAX, avgPositionError = 0.0, maxPositionError = DBL_MIN;
	double minRotationalError = DBL_MAX, avgRotationalError = 0.0, maxRotationalError = DBL_MIN;
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		serialDevice->setQ(measurements[measurementIndex]->getQ(), state);

		const Eigen::Affine3d tfmMeasurement = measurements[measurementIndex]->getPose().toTransform();
		const Eigen::Affine3d tfmModel = Eigen::Affine3d(rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state));
		const Eigen::Affine3d tfmError = tfmModel.difference(tfmMeasurement);

		double positionError = tfmError.translation().norm();
		if (positionError < minPositionError)
			minPositionError = positionError;
		avgPositionError += positionError / measurementCount;
		if (positionError > maxPositionError)
			maxPositionError = positionError;

		double rotationalError = Eigen::AngleAxisd(tfmError.linear()).angle();
		if (rotationalError < minRotationalError)
			minRotationalError = rotationalError;
		avgRotationalError += rotationalError / measurementCount;
		if (rotationalError > maxRotationalError)
			maxRotationalError = rotationalError;

		std::cout << "\tMeasurement " << measurementIndex + 1 << ": [ Positional: " << positionError * 100.0 << " cm - Rotational: "
				<< rotationalError * 180 / M_PI
				<< " ° ]" << std::endl;
	}
	std::cout << "\tSummary:" << std::endl;
	std::cout << "\tPositional [ Min: " << minPositionError * 100.0 << " cm - Avg: " << avgPositionError * 100.0 << " cm - Max: " << maxPositionError * 100.0
			<< " cm ]" << std::endl;
	std::cout << "\tRotational [ Min: " << minRotationalError * 180 / M_PI << " ° - Avg: " << avgRotationalError * 180 / M_PI << " ° - Max: "
			<< maxRotationalError * 180 / M_PI << " ° ]" << std::endl;
}

void printSolverLog() {
	std::vector<rwlibs::calibration::NLLSIterationLog> iterationLogs = serialDeviceCalibrator->getLog()->getIterationLogs();
	for (std::vector<rwlibs::calibration::NLLSIterationLog>::const_iterator it = iterationLogs.begin(); it != iterationLogs.end(); it++) {
		rwlibs::calibration::NLLSIterationLog iterationLog = *it;
		std::cout << "\tIteration " << iterationLog.getIterationNumber() << ": [ Singular: " << (iterationLog.isSingular() ? "Yes" : "No") << " - Condition: "
				<< iterationLog.getConditionNumber() << " - ||Residuals||: " << iterationLog.getResidualNorm() << " - ||Step||: " << iterationLog.getStepNorm()
				<< " ]" << std::endl;
	}
}

void printCalibrationSummary() {
	rwlibs::calibration::FixedFrameCalibration::Ptr baseCalibration = serialDeviceCalibration->getBaseCalibration();
	std::cout << "\tBase calibration of [ " << baseCalibration->getFrame()->getName() << " ]: [ Translation: "
			<< baseCalibration->getCorrection().translation().norm() * 100.0 << " cm - Rotation: "
			<< Eigen::AngleAxisd(baseCalibration->getCorrection().linear()).angle() * 180 / M_PI << " ° ]" << std::endl;

	rwlibs::calibration::FixedFrameCalibration::Ptr endCalibration = serialDeviceCalibration->getEndCalibration();
	std::cout << "\tEnd calibration of [ " << endCalibration->getFrame()->getName() << " ]: [ Translation: "
			<< endCalibration->getCorrection().translation().norm() * 100.0 << " cm - Rotation: "
			<< Eigen::AngleAxisd(endCalibration->getCorrection().linear()).angle() * 180 / M_PI << " ° ]" << std::endl;

	rwlibs::calibration::CompositeCalibration<rwlibs::calibration::DHParameterCalibration>::Ptr dhParameterCalibration =
			serialDeviceCalibration->getCompositeDHParameterCalibration();
	for (unsigned int calibrationIndex = 0; calibrationIndex < dhParameterCalibration->getCalibrations().size(); calibrationIndex++) {
		rwlibs::calibration::DHParameterCalibration::Ptr dhCalibration = dhParameterCalibration->getCalibrations()[calibrationIndex];
		Eigen::Vector4d correction = dhCalibration->getCorrection();
		std::cout << "\tDH calibration of [ " << dhCalibration->getJoint()->getName() << " ]: [ a: "
				<< correction(rwlibs::calibration::DHParameterCalibration::PARAMETER_A) * 100.0 << " cm - b/d: "
				<< correction(rwlibs::calibration::DHParameterCalibration::PARAMETER_B_D) * 100.0 << " cm - alpha: "
				<< correction(rwlibs::calibration::DHParameterCalibration::PARAMETER_ALPHA) * 180 / M_PI << " ° - beta/theta: "
				<< correction(rwlibs::calibration::DHParameterCalibration::PARAMETER_BETA_THETA) * 180 / M_PI << " ° ]" << std::endl;
	}
}
