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


#include "CollisionModelInfo.hpp"

using namespace rw::models;

CollisionModelInfo::CollisionModelInfo(const std::string& id, double scale):
	_colId(id),
	_transform(rw::math::Transform3D<>::identity()),
	_geoScale(scale)
{
}

CollisionModelInfo::CollisionModelInfo(const std::string& id,
			rw::math::Transform3D<> t3d, double scale):
	_colId(id),
	_transform(t3d),
	_geoScale(scale)
{
}
