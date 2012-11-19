#include "MultivariateNormalDistribution.hpp"
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
int measurementCount;
bool addNoise;

int parseArguments(int argumentCount, char** arguments);
void printHelp();
std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> generateMeasurements(rw::models::SerialDevice::Ptr serialDevice,
	rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, unsigned int measurementCount,
	bool addNoise);
void printMeasurementErrors(rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
	rw::kinematics::State state, const std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr>& measurements, rwlibs::calibration::Calibration::Ptr calibration);

int main(int argumentCount, char** arguments) {
	if (int parseResult = parseArguments(argumentCount, arguments) < 1)
		return parseResult;

	std::cout << "Initializing calibration measurement tool:" << std::endl;

	// Load workcell.
	std::cout << "\tLoading work cell [ " << workCellFilePath << " ].. ";
	std::cout.flush();
	rw::models::WorkCell::Ptr workCell = rw::loaders::XMLRWLoader::load(workCellFilePath);
	if (workCell.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Loaded [ " << workCell->getName() << " ]." << std::endl;
	rw::kinematics::State state = workCell->getDefaultState();

	// Find device and frames.
	std::cout << "\tFinding device [ " << deviceName << " ].. ";
	std::cout.flush();
	rw::models::Device::Ptr device = deviceName.empty() ? workCell->getDevices().front() : workCell->findDevice(deviceName);
	if (device.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Found [ " << device->getName() << " ]." << std::endl;
	rw::models::SerialDevice::Ptr serialDevice = device.cast<rw::models::SerialDevice>();

	std::cout << "\tFinding reference frame [ " << referenceFrameName << " ].. ";
	std::cout.flush();
	rw::kinematics::Frame::Ptr referenceFrame = referenceFrameName.empty() ? workCell->findFrame("WORLD") : workCell->findFrame(referenceFrameName);
	if (referenceFrame.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Found [ " << referenceFrame->getName() << " ]." << std::endl;

	std::cout << "\tFinding measurement frame [ " << measurementFrameName << " ].. ";
	std::cout.flush();
	rw::kinematics::Frame::Ptr measurementFrame = measurementFrameName.empty() ? device->getEnd() : workCell->findFrame(measurementFrameName);
	if (measurementFrame.isNull()) {
		std::cout << "FAILED." << std::endl;
		return -1;
	}
	std::cout << "Found [ " << measurementFrame->getName() << " ]." << std::endl;

	std::cout << "Initializing artificial calibration..";
	rwlibs::calibration::SerialDeviceCalibration::Ptr artificialCalibration(rw::common::ownedPtr(new rwlibs::calibration::SerialDeviceCalibration(serialDevice)));
	artificialCalibration->getBaseCalibration()->setCorrection(rw::math::Transform3D<>(rw::math::Vector3D<>(0.07, 0.008, 0.009), rw::math::RPY<>(0.08, 0.007, 0.06)));
	artificialCalibration->getEndCalibration()->setCorrection(rw::math::Transform3D<>(rw::math::Vector3D<>(0.01, 0.002, 0.003), rw::math::RPY<>(0.04, 0.005, 0.06)));
	std::vector<rwlibs::calibration::DHParameterCalibration::Ptr> artificialDhParameterCalibrations =
		artificialCalibration->getCompositeDHParameterCalibration()->getCalibrations();
	for (unsigned int calibrationIndex = 0; calibrationIndex < artificialDhParameterCalibrations.size(); calibrationIndex++)
		artificialDhParameterCalibrations[calibrationIndex]->setCorrection(Eigen::Vector4d(0.003, 0.0, -0.002, 0.0));
	std::cout << " Initialized." << std::endl;

	std::cout << "Applying artificial calibration..";
	artificialCalibration->apply();
	std::cout << " Applied." << std::endl;

	std::cout << "Generating measurements [ Count: " << measurementCount << " - Noise: " << (addNoise ? "Enabled" : "Disabled") << " ]..";
	std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> measurements = generateMeasurements(serialDevice, referenceFrame, measurementFrame,
		state, measurementCount, addNoise);
	std::cout << " Generated." << std::endl;

	std::cout << "Reverting artificial calibration..";
	artificialCalibration->revert();
	std::cout << " Reverted." << std::endl;

	std::cout << "Saving measurement file [ " << measurementFilePath << " ]..";
	rwlibs::calibration::XmlMeasurementFile::save(measurements, measurementFilePath);
	std::cout << " Saved." << std::endl;

	std::cout << "Residual summary:" << std::endl;
	printMeasurementErrors(serialDevice, referenceFrame, measurementFrame, state, measurements, artificialCalibration);

	return 0;
}

int parseArguments(int argumentCount, char** arguments) {
	optionsDescription.add_options()("help", "Print help message")("workCellFile", boost::program_options::value<std::string>(&workCellFilePath)->required(),
		"Set the work cell file path")("measurementFile", boost::program_options::value<std::string>(&measurementFilePath)->required(), "Set the measurement file path")("device",
		boost::program_options::value<std::string>(&deviceName), "Set the device name")("referenceFrame",
		boost::program_options::value<std::string>(&referenceFrameName), "Set the reference frame name")("measurementFrame",
		boost::program_options::value<std::string>(&measurementFrameName), "Set the measurement frame name")("measurementCount",
		boost::program_options::value<int>(&measurementCount)->default_value(50), "Set the number of measurements")("addNoise", boost::program_options::value<bool>(&addNoise)->default_value(true), "Enable/disable noise on measurements");

	positionalOptionsDescription.add("workCellFile", 1).add("measurementFile", 1);

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
	std::cerr << "  rw_calibration-mtool [work cell file] [measurement file]" << std::endl;
	std::cerr << std::endl;
	std::cerr << optionsDescription << std::endl;
}

std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> generateMeasurements(rw::models::SerialDevice::Ptr serialDevice,
	rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, unsigned int measurementCount,
	bool addNoise) {
		MultivariateNormalDistribution<double, 6> mvnd(time(0));

		std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> measurements(measurementCount);
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
				covariance /= 10e7;

				Eigen::Matrix<double, 6, 1> mvndVector = mvnd.draw(covariance);
				transform.P() = rw::math::Vector3D<>(mvndVector(0), mvndVector(1), mvndVector(2)) + transform.P();
				transform.R() = rw::math::RPY<>(mvndVector(3), mvndVector(4), mvndVector(5)).toRotation3D() * transform.R();
			}

			measurements[measurementIndex] = rw::common::ownedPtr(new rwlibs::calibration::SerialDevicePoseMeasurement(q, transform, covariance));
		}

		return measurements;
}

void printMeasurementErrors(rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
	rw::kinematics::State state, const std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr>& measurements, rwlibs::calibration::Calibration::Ptr calibration) {
		const unsigned int measurementCount = measurements.size();

		Eigen::VectorXd distances(measurementCount), angles(measurementCount);
		Eigen::VectorXd calibratedDistances(measurementCount), calibratedAngles(measurementCount);
		for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
			serialDevice->setQ(measurements[measurementIndex]->getQ(), state);

			const rw::math::Transform3D<> tfmMeasurement = measurements[measurementIndex]->getTransform();
			const rw::math::Transform3D<> tfmModel = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state);
			calibration->apply();
			const rw::math::Transform3D<> tfmCalibratedModel =
				rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state);
			calibration->revert();
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
