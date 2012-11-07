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
bool isWeightingEnabled = true;
bool isBaseCalibrationEnabled = true;
bool isEndCalibrationEnabled = true;
bool isLinkCalibrationEnabled = true;

rw::models::WorkCell::Ptr workCell;
rw::kinematics::State state;
rw::models::Device::Ptr device;
rw::models::SerialDevice::Ptr serialDevice;
rw::kinematics::Frame::Ptr referenceFrame;
rw::kinematics::Frame::Ptr measurementFrame;
std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> measurements;

rwlibs::calibration::SerialDeviceCalibration::Ptr serialDeviceCalibration;
rwlibs::calibration::SerialDeviceCalibrator::Ptr serialDeviceCalibrator;

int parseArguments(int argumentCount, char** arguments);
void printHelp();
void printMeasurements();
void printSolverLog();
void printCalibrationSummary();

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
	measurements = rwlibs::calibration::XmlMeasurementFile::load(measurementFilePath);
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
	serialDeviceCalibration = rw::common::ownedPtr(new rwlibs::calibration::SerialDeviceCalibration(serialDevice));
	serialDeviceCalibration->getBaseCalibration()->setLocked(!isBaseCalibrationEnabled);
	serialDeviceCalibration->getEndCalibration()->setLocked(!isEndCalibrationEnabled);
	serialDeviceCalibration->getCompositeDHParameterCalibration()->setLocked(!isLinkCalibrationEnabled);
	std::cout << "Initialized [ Base calibration: " << (!serialDeviceCalibration->getBaseCalibration()->isLocked() ? "Enabled" : "Disabled")
			<< " - End calibration: " << (!serialDeviceCalibration->getEndCalibration()->isLocked() ? "Enabled" : "Disabled") << " - Link calibration: "
			<< (!serialDeviceCalibration->getCompositeDHParameterCalibration()->isLocked() ? "Enabled" : "Disabled") << " ]." << std::endl;

	std::cout << "\tInitializing calibrator..";
	std::cout.flush();
	serialDeviceCalibrator = rw::common::ownedPtr(
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
		printSolverLog();
		std::cout << "\tSucceeded." << std::endl;
	} catch (rw::common::Exception& exception) {
		printSolverLog();
		std::cout << "\tFAILED: " << exception.getMessage() << std::endl;
	}

	// Print calibration summary.
	std::cout << "Calibration summary:" << std::endl;
	printCalibrationSummary();

	// Print differences between model and measurements.
	std::cout << "Residual summary:" << std::endl;
	printMeasurements();

	// Save calibration.
	if (!calibrationFilePath.empty()) {
		std::cout << "Saving calibration [" << calibrationFilePath << "].. ";
		std::cout.flush();
		rwlibs::calibration::XmlCalibrationSaver::save(serialDeviceCalibration, calibrationFilePath);
		std::cout << "Saved." << std::endl;
	}

	return 0;
}

int parseArguments(int argumentCount, char** arguments) {
	optionsDescription.add_options()("help", "Print help message")("workCellFile", boost::program_options::value<std::string>(&workCellFilePath)->required(),
			"Set the work cell file path")("measurementFile", boost::program_options::value<std::string>(&measurementFilePath)->required(), "Set the measurement file path")(
			"calibrationFile", boost::program_options::value<std::string>(&calibrationFilePath), "Set the calibration file path")("device",
			boost::program_options::value<std::string>(&deviceName), "Set the device name")("referenceFrame",
			boost::program_options::value<std::string>(&referenceFrameName), "Set the reference frame name")("measurementFrame",
			boost::program_options::value<std::string>(&measurementFrameName), "Set the measurement frame name")("disableWeighting",
			"Disable measurement weighting")("disableBaseCalibration", "Disable calibration of base transformation")("disableEndCalibration",
			"Disable calibration of end transformation")("disableLinkCalibration", "Disable calibration of link transformations");

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

	if (variablesMap.count("disableWeighting"))
		isWeightingEnabled = false;

	if (variablesMap.count("disableBaseCalibration"))
		isBaseCalibrationEnabled = false;
	if (variablesMap.count("disableEndCalibration"))
		isEndCalibrationEnabled = false;
	if (variablesMap.count("disableLinkCalibration"))
		isLinkCalibrationEnabled = false;

	return 1;
}

void printHelp() {
	std::cerr << "Usage:" << std::endl;
	std::cerr << "  rw_calibration-tool [options] [work cell file] [measurement file] [calibration file]" << std::endl;
	std::cerr << std::endl;
	std::cerr << optionsDescription << std::endl;
}

void printMeasurements() {
	const unsigned int measurementCount = measurements.size();

	Eigen::VectorXd distances(measurementCount), angles(measurementCount);
	Eigen::VectorXd calibratedDistances(measurementCount), calibratedAngles(measurementCount);
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		serialDevice->setQ(measurements[measurementIndex]->getQ(), state);

		const Eigen::Affine3d tfmMeasurement = measurements[measurementIndex]->getPose().toTransform();
		const Eigen::Affine3d tfmModel = Eigen::Affine3d(rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state));
		serialDeviceCalibration->apply();
		const Eigen::Affine3d tfmCalibratedModel = Eigen::Affine3d(
				rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state));
		serialDeviceCalibration->revert();
		const Eigen::Affine3d tfmError = tfmModel.difference(tfmMeasurement);
		const Eigen::Affine3d tfmCalibratedError = tfmCalibratedModel.difference(tfmMeasurement);

		distances(measurementIndex) = tfmError.translation().norm(), calibratedDistances(measurementIndex) = tfmCalibratedError.translation().norm();
		angles(measurementIndex) = Eigen::AngleAxisd(tfmError.linear()).angle(), calibratedAngles(measurementIndex) = Eigen::AngleAxisd(
				tfmCalibratedError.linear()).angle();

		std::cout << "\tMeasurement " << measurementIndex + 1 << ": [ Before: " << distances(measurementIndex) * 100.0 << " cm / "
				<< angles(measurementIndex) * 180 / M_PI << " ° - After: " << calibratedDistances(measurementIndex) * 100.0 << " cm / "
				<< calibratedAngles(measurementIndex) * 180 / M_PI << " ° ]" << std::endl;
	}
	std::cout << "\tSummary - Before: [ Avg: " << distances.mean() * 100.0 << " cm / " << angles.mean() * 180 / M_PI << " ° - Min: "
			<< distances.minCoeff() * 100.0 << " cm / " << angles.minCoeff() * 180 / M_PI << " ° - Max: " << distances.maxCoeff() * 100.0 << " cm / "
			<< angles.maxCoeff() * 180 / M_PI << " ° ]" << std::endl;
	std::cout << "\tSummary - After: [ Avg: " << calibratedDistances.mean() * 100.0 << " cm / " << calibratedAngles.mean() * 180 / M_PI << " ° - Min: "
			<< calibratedDistances.minCoeff() * 100.0 << " cm / " << calibratedAngles.minCoeff() * 180 / M_PI << " ° - Max: "
			<< calibratedDistances.maxCoeff() * 100.0 << " cm / " << calibratedAngles.maxCoeff() * 180 / M_PI << " ° ]" << std::endl;
}

void printSolverLog() {
	std::vector<rwlibs::calibration::NLLSIterationLog> iterationLogs = serialDeviceCalibrator->getSolverLog()->getIterationLogs();
	for (std::vector<rwlibs::calibration::NLLSIterationLog>::const_iterator it = iterationLogs.begin(); it != iterationLogs.end(); it++) {
		rwlibs::calibration::NLLSIterationLog iterationLog = *it;
		std::cout << "\tIteration " << iterationLog.getIterationNumber() << ": Jacobian [ Singular: " << (iterationLog.isSingular() ? "Yes" : "No")
				<< " - Condition: " << iterationLog.getConditionNumber() << " ] - ||Residuals||: " << iterationLog.getResidualNorm() << " - ||Step||: "
				<< iterationLog.getStepNorm() << " ]" << std::endl;
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

std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> generateMeasurements(rw::models::SerialDevice::Ptr serialDevice,
		rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, unsigned int measurementCount,
		bool addNoise) {
	MultivariateNormalDistribution<double, 6> mvnd(time(0));

	std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> measurements(measurementCount);
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		rw::math::Q q = rw::math::Math::ranQ(serialDevice->getBounds());
		serialDevice->setQ(q, state);

		rwlibs::calibration::Pose6Dd pose(rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state));

		Eigen::Affine3d transform = pose.toTransform();
		Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
		if (addNoise) {
			Eigen::Matrix<double, 6, 6> random = Eigen::Matrix<double, 6, 6>::Random();
			covariance = random.transpose() * random;
			covariance.block<3, 3>(0, 0) /= 50.0;
			covariance.block<3, 3>(3, 3) /= 5.0;
			covariance.block<3, 3>(3, 0) /= 1000.0;
			covariance.block<3, 3>(0, 3) /= 1000.0;
			covariance /= 10e14;

			rwlibs::calibration::Pose6Dd poseNoise = rwlibs::calibration::Pose6Dd(mvnd.draw(covariance));
			Eigen::Affine3d noise = poseNoise.toTransform();
			transform.linear() = noise.linear() * transform.linear();
			transform.translation() = noise.translation() + transform.translation();
		}
		rwlibs::calibration::Pose6Dd noisyPose = rwlibs::calibration::Pose6Dd(transform);

		measurements[measurementIndex] = rw::common::ownedPtr(new rwlibs::calibration::SerialDevicePoseMeasurement(q, noisyPose, covariance));
	}

	return measurements;
}
