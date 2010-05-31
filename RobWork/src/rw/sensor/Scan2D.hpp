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


#ifndef RW_SENSOR_SCAN2D_HPP
#define RW_SENSOR_SCAN2D_HPP

#include "SensorData.hpp"
#include <rw/math/Vector3D.hpp>
#include <vector>

namespace rw {
namespace sensor {

/** @addtogroup sensor */
/*@{*/

/**
 * @brief data structure for range scanner data
 */
class Scan2D: public SensorData {
public:
	Scan2D();
	virtual ~Scan2D();

    /**
     * @brief resizes the current scan.
     * @param width
     */
    void resize(int width){
        _width = width;
        _data.resize(_width);
    }

    /**
     * @brief returns a char pointer to the image data
     * @return char pointer to the image data
     */
    std::vector<rw::math::Vector3D<float> >& getImageData() { return _data; };

    /**
     * @brief returns a char pointer to the image data
     * @return const char pointer to the image data
     */
    const std::vector<rw::math::Vector3D<float> >& getImageData() const{ return _data; };

    /**
     * @brief returns the width of this image
     * @return image width
     */
    unsigned int getWidth() const { return _width;};


private:
    size_t _width;
	std::vector<rw::math::Vector3D<float> > _data;

	//std::vector<float> _angle;
    //std::vector<float> _depth;
};

/*@}*/

} //end namespace sensor
} //end namespace rw

#endif /*RW_SENSOR_SCAN2D_HPP*/
