/*
 * SerialDevicePoseMeasurements.hpp
 *
 *  Created on: May 22, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENTLIST_HPP
#define RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENTLIST_HPP

#include "SerialDevicePoseMeasurement.hpp"

#include <rw/common.hpp>

namespace rwlibs {
namespace calibration {

class SerialDevicePoseMeasurementList: public std::vector<SerialDevicePoseMeasurement, Eigen::aligned_allocator<SerialDevicePoseMeasurement> > {
public:
	void save(std::string fileName);

	static SerialDevicePoseMeasurementList load(std::string fileName);
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICEPOSEMEASUREMENTLIST_HPP */
