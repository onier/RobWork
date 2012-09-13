/*
 * SerialDevicePoseMeasurements.hpp
 *
 *  Created on: May 22, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENTLIST_HPP
#define RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENTLIST_HPP

#include "SerialDevicePoseMeasurement.hpp"

namespace rwlibs {
namespace calibration {

class SerialDevicePoseMeasurementList {
public:
	static void save(const std::vector<SerialDevicePoseMeasurement::Ptr>& measurements, std::string fileName);

	static std::vector<SerialDevicePoseMeasurement::Ptr> load(std::string fileName);
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENTLIST_HPP */
