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


#ifndef GEOMETRY_HPP_
#define GEOMETRY_HPP_

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/common/Ptr.hpp>
#include "GeometryData.hpp"

namespace rw { namespace geometry {

class Geometry {

public:

    Geometry(GeometryDataPtr data, double scale=1.0):
        _data(data),
        _transform(rw::math::Transform3D<>::identity() ),
        _scale(scale)
    {
    };

    Geometry(GeometryDataPtr data,
             const rw::math::Transform3D<>& t3d,
             double scale=1.0):

        _data(data),
        _transform(t3d),
        _scale(scale)
    {
    };

    virtual ~Geometry(){};

    /**
     * @brief gets the geometric scale of the collision model.
     */
    double getScale() const {
        return _scale;
    }

    void setScale(double scale){
        _scale = scale;
    }

    void setTransform(const rw::math::Transform3D<>& t3d){_transform = t3d;};
    const rw::math::Transform3D<>& getTransform(){return _transform;};

    GeometryDataPtr getGeometryData(){return _data;};
    void setGeometryData(GeometryDataPtr data){_data = data;};

    GeometryData* getBV(){return _bv;};
    void setBV(GeometryData* bv){_bv = bv;};

private:

    GeometryDataPtr _data;
    GeometryData *_bv;
    rw::math::Transform3D<> _transform;
    double _scale;

};

typedef rw::common::Ptr<Geometry> GeometryPtr;

}
}

#endif /* GEOMETRY_HPP_ */
