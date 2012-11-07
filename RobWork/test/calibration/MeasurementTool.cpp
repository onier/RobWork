#include "MultivariateNormalDistribution.hpp"
#include <rw/loaders.hpp>
#include <rwlibs/calibration.hpp>

std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> generateMeasurements(rw::models::SerialDevice::Ptr serialDevice,
		rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, int measurementCount,
		bool addNoise);

void printMeasurementErrors(rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
		rw::kinematics::State state, const std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr>& measurements);

int main(int argumentCount, char** arguments) {
	if (argumentCount < 7)
		RW_THROW("Not enough arguments.");

	const std::string workCellFilePath(arguments[1]);
	const std::string deviceName(arguments[2]);
	const std::string referenceFrameName(arguments[3]);
	const std::string measurementFrameName(arguments[4]);
	const std::string measurementFilePath(arguments[5]);
	const int measurementCount(std::atoi(arguments[6]));
	const int addNoise(std::atoi(arguments[7]));

	// Load workcell.
	std::cout << "Loading work cell (" << workCellFilePath << ").." << std::endl;
	rw::models::WorkCell::Ptr workCell = rw::loaders::XMLRWLoader::load(workCellFilePath);
	if (workCell.isNull())
		RW_THROW("Work cell not loaded.");
	rw::kinematics::State state = workCell->getDefaultState();

	// Find device and frames.
	std::cout << "Finding device (" << deviceName << ").." << std::endl;
	rw::models::Device::Ptr device = deviceName.empty() ? workCell->getDevices().front() : workCell->findDevice(deviceName);
	if (device.isNull())
		RW_THROW("Device not found.");
	rw::models::SerialDevice::Ptr serialDevice = device.cast<rw::models::SerialDevice>();
	std::cout << "Finding reference frame (" << referenceFrameName << ").." << std::endl;
	rw::kinematics::Frame::Ptr referenceFrame = referenceFrameName.empty() ? workCell->findFrame("WORLD") : workCell->findFrame(referenceFrameName);
	if (referenceFrame.isNull())
		RW_THROW("Reference frame not found.");
	std::cout << "Finding measurement frame (" << measurementFrameName << ").." << std::endl;
	rw::kinematics::Frame::Ptr measurementFrame = measurementFrameName.empty() ? device->getEnd() : workCell->findFrame(measurementFrameName);
	if (measurementFrame.isNull())
		RW_THROW("Measurement frame not found.");

	std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> measurements = generateMeasurements(serialDevice, referenceFrame, measurementFrame,
			state, measurementCount, addNoise);

	printMeasurementErrors(serialDevice, referenceFrame, measurementFrame, state, measurements);

	std::cout << "Saving measurement file (" << measurementFilePath << ").." << std::endl;
	rwlibs::calibration::XmlMeasurementFile::save(measurements, measurementFilePath);
	std::cout << "Saved." << std::endl;

	return 0;
}

std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> generateMeasurements(rw::models::SerialDevice::Ptr serialDevice,
		rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, int measurementCount,
		bool addNoise) {
	rwlibs::calibration::SerialDeviceCalibration calibration(serialDevice);
	calibration.getBaseCalibration()->setCorrection(rwlibs::calibration::Pose6Dd(0.007, 0.0008, 0.09, 0.08, 0.007, 0.06).toTransform());
	calibration.getEndCalibration()->setCorrection(rwlibs::calibration::Pose6Dd(0.001, 0.0002, 0.03, 0.04, 0.005, 0.06).toTransform());
	/*std::vector<rwlibs::calibration::DHParameterCalibration::Ptr> dhParameterCalibrations = calibration.getCompositeDHParameterCalibration()->getCalibrations();
	 for (int calibrationIndex = 0; calibrationIndex < dhParameterCalibrations.size(); calibrationIndex++) {
	 rw::models::DHParameterSet dhParameterSet = dhParameterCalibrations[calibrationIndex]->getCorrection();
	 }*/

	MultivariateNormalDistribution<double, 6> mvnd(time(0));

	std::cout << "Generating " << measurementCount << " measurements.." << std::endl;
	std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr> measurements(measurementCount);
	for (int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		std::cout << "Generating measurement " << measurementIndex + 1 << "..";

		rw::math::Q q = rw::math::Math::ranQ(serialDevice->getBounds());
		serialDevice->setQ(q, state);

		calibration.apply();
		rwlibs::calibration::Pose6Dd pose(rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state));
		calibration.revert();

		Eigen::Affine3d transform = pose.toTransform();
		Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
		if (addNoise) {
			Eigen::Matrix<double, 6, 6> random = Eigen::Matrix<double, 6, 6>::Random();
			covariance = random.transpose() * random;
			covariance.block<3, 3>(0, 0) /= 5000.0;
			covariance.block<3, 3>(3, 3) /= 500.0;
			covariance.block<3, 3>(3, 0) /= 100000.0;
			covariance.block<3, 3>(0, 3) /= 100000.0;
			covariance /= 10e4;

			rwlibs::calibration::Pose6Dd poseNoise = rwlibs::calibration::Pose6Dd(mvnd.draw(covariance));
			Eigen::Affine3d noise = poseNoise.toTransform();
			transform.linear() = noise.linear() * transform.linear();
			transform.translation() = noise.translation() + transform.translation();

			std::cout << " - Noise [ Position: " << noise.translation().norm() * 100.0 << " cm - Rotation: "
					<< Eigen::AngleAxisd(noise.linear()).angle() * 180 / M_PI << " °]";
		}
		rwlibs::calibration::Pose6Dd noisyPose = rwlibs::calibration::Pose6Dd(transform);

		measurements[measurementIndex] = rw::common::ownedPtr(new rwlibs::calibration::SerialDevicePoseMeasurement(q, noisyPose, covariance));

		const Eigen::Affine3d tfmMeasurement = measurements[measurementIndex]->getPose().toTransform();
		const Eigen::Affine3d tfmModel = Eigen::Affine3d(rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state));
		Eigen::Affine3d error;
		error.translation() = tfmModel.translation() - tfmMeasurement.translation();
		error.linear() = tfmModel.linear() * tfmMeasurement.rotation().transpose();
		std::cout << " - Error [ Position: " << error.translation().norm() * 100.0 << " cm - Rotation: "
				<< Eigen::AngleAxisd(error.linear()).angle() * 180 / M_PI << " °]";

		std::cout << std::endl;
	}

	return measurements;
}

void printMeasurementErrors(rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
		rw::kinematics::State state, const std::vector<rwlibs::calibration::SerialDevicePoseMeasurement::Ptr>& measurements) {
	const unsigned int measurementCount = measurements.size();

	Eigen::VectorXd distances(measurementCount), angles(measurementCount);
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		serialDevice->setQ(measurements[measurementIndex]->getQ(), state);

		const Eigen::Affine3d tfmMeasurement = measurements[measurementIndex]->getPose().toTransform();
		const Eigen::Affine3d tfmModel = Eigen::Affine3d(rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state));
		const Eigen::Affine3d tfmError = tfmModel.difference(tfmMeasurement);

		distances(measurementIndex) = tfmError.translation().norm();
		angles(measurementIndex) = Eigen::AngleAxisd(tfmError.linear()).angle();

		std::cout << "\tMeasurement " << measurementIndex + 1 << ": [ " << distances(measurementIndex) * 100.0 << " cm / "
				<< angles(measurementIndex) * 180 / M_PI << " ° ]" << std::endl;
	}
	std::cout << "\tSummary: [ Avg: " << distances.mean() * 100.0 << " cm / " << angles.mean() * 180 / M_PI << " ° - Min: "
			<< distances.minCoeff() * 100.0 << " cm / " << angles.minCoeff() * 180 / M_PI << " ° - Max: " << distances.maxCoeff() * 100.0 << " cm / "
			<< angles.maxCoeff() * 180 / M_PI << " ° ]" << std::endl;
}
