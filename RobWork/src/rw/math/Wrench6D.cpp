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


#include "Wrench6D.hpp"

using namespace rw::math;

template<class T>
Wrench6D<T>::Wrench6D(T vx, T vy, T vz, T wx, T wy, T wz) : _wrench(6){
    _wrench[0] = vx;
    _wrench[1] = vy;
    _wrench[2] = vz;
    _wrench[3] = wx;
    _wrench[4] = wy;
    _wrench[5] = wz;
}


template class Wrench6D<double>;
template class Wrench6D<float>;

