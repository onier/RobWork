#include <rw/loaders.hpp>
#include <rwlibs/calibration.hpp>

std::string workCellFilePath;
std::string deviceName;
std::string referenceFrameName;
std::string measurementFrameName;
std::string measurementFilePath;
std::string calibrationFilePath;
bool weighted;
bool enableBaseCalibration;
bool enableEndCalibration;
bool enableDHCalibration;

rw::models::WorkCell::Ptr workCell;
rw::kinematics::State state;
rw::models::Device::Ptr device;
rw::models::SerialDevice::Ptr serialDevice;
rw::kinematics::Frame::Ptr referenceFrame;
rw::kinematics::Frame::Ptr measurementFrame;
std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> measurements;

rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibration;
rwlibs::calibration::SerialDeviceCalibrator::Ptr serialDeviceCalibrator;

void initializeTool(int argumentCount, char** arguments);
void printMeasurementSummary();
void printSolverLog();
void printCalibrationSummary();

int main(int argumentCount, char** arguments) {
	try {
		std::cout << "<!--" << std::endl;
		std::cout << "Initializing calibration tool.." << std::endl;
		initializeTool(argumentCount, arguments);
		std::cout << "Initialized." << std::endl;

		// Print differences between model and measurements before calibration.
		std::cout << "Printing summary of differences between model and measurements before calibration.." << std::endl;
		printMeasurementSummary();
		std::cout << "Printed." << std::endl;

		// Run calibrator.
		std::cout << "Calibrating..";
		std::cout.flush();
		serialDeviceCalibrator->calibrate();
		std::cout << " Calibrated in " << serialDeviceCalibrator->getLog()->getIterationLogs().size() << " iterations." << std::endl;

		std::cout << "Printing solver log.." << std::endl;
		printSolverLog();
		std::cout << "Printed." << std::endl;

		// Print calibration summary.
		std::cout << "Printing calibration summary.." << std::endl;
		printCalibrationSummary();
		std::cout << "Printed." << std::endl;

		// Apply calibration.
		std::cout << "Applying calibration..";
		std::cout.flush();
		serialDeviceCalibration->apply();
		std::cout << " Applied." << std::endl;

		// Print differences between model and measurements after calibration.
		std::cout << "Printing differences between model and measurements after calibration.." << std::endl;
		printMeasurementSummary();
		std::cout << "Printed." << std::endl;

		// Save and load calibration.
		std::cout << "Printing calibration [" << calibrationFilePath << "].." << std::endl;
		std::cout << "-->" << std::endl;
	} catch (rw::common::Exception& exception) {
		std::cout << "-->" << std::endl;
		throw exception;
	}

	rwlibs::calibration::XmlCalibrationSaver::save(serialDeviceCalibration, std::cout);

	return 0;
}

void initializeTool(int argumentCount, char** arguments) {
	if (argumentCount < 7)
		RW_THROW("Not enough arguments.");

	workCellFilePath = arguments[1];
	deviceName = arguments[2];
	referenceFrameName = arguments[3];
	measurementFrameName = arguments[4];
	measurementFilePath = arguments[5];
	calibrationFilePath = arguments[6];
	weighted = argumentCount >= 7 ? std::atoi(arguments[7]) : true;
	enableBaseCalibration = argumentCount >= 8 ? std::atoi(arguments[8]) : true;
	enableEndCalibration = argumentCount >= 9 ? std::atoi(arguments[9]) : true;
	enableDHCalibration = argumentCount >= 10 ? std::atoi(arguments[10]) : true;

	// Load workcell.
	std::cout << "\tLoading work cell [ " << workCellFilePath << " ]..";
	std::cout.flush();
	workCell = rw::loaders::XMLRWLoader::load(workCellFilePath);
	if (workCell.isNull())
		RW_THROW("Work cell not loaded.");
	state = workCell->getDefaultState();
	std::cout << " Loaded." << std::endl;

	// Find device and frames.
	std::cout << "\tFinding device [ " << deviceName << " ]..";
	std::cout.flush();
	device = workCell->findDevice(deviceName);
	if (device.isNull())
		RW_THROW("Device not found.");
	serialDevice = device.cast<rw::models::SerialDevice>();
	std::cout << " Found." << std::endl;

	std::cout << "\tFinding reference frame [ " << referenceFrameName << " ]..";
	std::cout.flush();
	referenceFrame = workCell->findFrame(referenceFrameName);
	if (referenceFrame.isNull())
		RW_THROW("Reference frame not found.");
	std::cout << " Found." << std::endl;

	std::cout << "\tFinding measurement frame [ " << measurementFrameName << " ]..";
	std::cout.flush();
	measurementFrame = workCell->findFrame(measurementFrameName);
	if (measurementFrame.isNull())
		RW_THROW("Measurement frame not found.");
	std::cout << " Found." << std::endl;

	// Load robot pose measurements from file.
	std::cout << "\tLoading measurements [ " << measurementFilePath << " ]..";
	std::cout.flush();
	measurements = rwlibs::calibration::XmlMeasurementFile::load(measurementFilePath);
	std::cout << " Loaded " << measurements.size() << " measurements." << std::endl;

	// Disable existing calibration if one exist.
	std::cout << "\tFinding existing calibration..";
	std::cout.flush();
	rwlibs::calibration::SerialDeviceCalibration::Ptr calibrationExisting = rwlibs::calibration::SerialDeviceCalibration::get(serialDevice);
	if (calibrationExisting.isNull())
		std::cout << " Not found." << std::endl;
	else {
		std::cout << " Found." << std::endl;

		std::cout << "\t - Disabling existing calibration..";
		std::cout.flush();
		calibrationExisting->revert();
		std::cout << " Disabled." << std::endl;
	}

	// Initialize calibration, jacobian and calibrator.
	std::cout << "\tInitializing calibration [ Base calibration: " << (enableBaseCalibration ? "Enabled" : "Disabled") << " - End calibration: "
			<< (enableEndCalibration ? "Enabled" : "Disabled") << " - DH calibration: " << (enableDHCalibration ? "Enabled" : "Disabled") << " ]..";
	std::cout.flush();
	serialDeviceCalibration = rw::common::ownedPtr(new rwlibs::calibration::SerialDeviceCalibration(serialDevice));
	serialDeviceCalibration->getBaseCalibration()->setLocked(!enableBaseCalibration);
	serialDeviceCalibration->getEndCalibration()->setLocked(!enableEndCalibration);
	serialDeviceCalibration->getCompositeDHParameterCalibration()->setLocked(!enableDHCalibration);
	std::cout << " Initialized." << std::endl;

	std::cout << "\tInitializing calibrator [ Weighted: " << (weighted ? "Yes" : "No") << " ]..";
	std::cout.flush();
	serialDeviceCalibrator = rw::common::ownedPtr(
			new rwlibs::calibration::SerialDeviceCalibrator(serialDevice, state, referenceFrame, measurementFrame, serialDeviceCalibration));
	if (measurements.size() < serialDeviceCalibrator->getMinimumMeasurementCount())
		RW_THROW("Not enough measurements.");
	serialDeviceCalibrator->setMeasurements(measurements);
	serialDeviceCalibrator->setWeighted(weighted);
	std::cout << " Initialized." << std::endl;
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
				<< rotationalError * 180 / M_PI << " ° ]" << std::endl;
	}
	std::cout << "\t-- Summary --" << std::endl;
	std::cout << "\tPositional [ Min: " << minPositionError * 100.0 << " cm - Avg: " << avgPositionError * 100.0 << " cm - Max: " << maxPositionError * 100.0
			<< " cm ]" << " - Rotational [ Min: " << minRotationalError * 180 / M_PI
			<< " ° - Avg: " << avgRotationalError * 180 / M_PI << " ° - Max: " << maxRotationalError * 180 / M_PI << " ° ]" << std::endl;
}

void printSolverLog() {
	std::vector<rwlibs::calibration::NLLSIterationLog> iterationLogs = serialDeviceCalibrator->getLog()->getIterationLogs();
	for (std::vector<rwlibs::calibration::NLLSIterationLog>::const_iterator it = iterationLogs.begin(); it != iterationLogs.end(); it++) {
		rwlibs::calibration::NLLSIterationLog iterationLog = *it;
		std::cout << "\tIteration " << iterationLog.getIterationNumber() << ": [ Singular: " << (iterationLog.isSingular() ? "Yes" : "No") << " - Condition: "
				<< iterationLog.getConditionNumber() << " - ||Residuals||: " << iterationLog.getResidualNorm() << " - ||Step||: " << iterationLog.getStepNorm()
				<< " ]." << std::endl;
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
