/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef RW_SENSOR_SCANNER2D_HPP
#define RW_SENSOR_SCANNER2D_HPP

/**
 * @file Scanner2D.hpp
 */

#include "Scanner.hpp"
#include "Scan2D.hpp"

namespace rw {
namespace sensor {

    /** @addtogroup sensor */
    /* @{ */

/**
 * @brief The Scanner2D sensor encapsulate the basic interface of a
 * 2 dimensional range scanning device such as SICK or Hokyuo laser
 * range scanners.
 *
 *  The interface supports any range scanner that measures distance in
 * an arc around the origin of the sensor.
 */

class Scanner2D: public Scanner {

protected:

    /**
     * @brief constructor
     * @param frame [in] the frame that the scanner is attached to
     * @param name [in] name of scanner sensor
     */
    Scanner2D(const std::string& name, const std::string& description = ""):
        Scanner(name, description)
    {
    }

public:
    /**
     * @brief destructor
     */
    virtual ~Scanner2D();

    /**
     * @brief gets the last acquired scan
     */
    virtual const Scan2D& getData() = 0;

    /**
     * @brief gets the scanning resolution in radians. The resolution
     * is the distance in radians between two consecutive data pixels.
     * @return the resolution of the scan.
     */
    virtual double getResolution() = 0;
};


/**
 * @brief Smart pointer to Scanner2D
 */
typedef rw::common::Ptr<Scanner2D> Scanner2DPtr;

/*@}*/

}
}

#endif /*RW_SENSOR_SCANNER2D_HPP_*/
