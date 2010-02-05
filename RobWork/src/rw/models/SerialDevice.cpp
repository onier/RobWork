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


#include "SerialDevice.hpp"
#include "Accessor.hpp"
#include "Joint.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::common;

namespace
{
    bool isActiveJoint(const Frame& frame)
    {
        return Accessor::activeJoint().has(frame);
    }

    bool isDependentJoint(const Frame& frame) {
        return Accessor::dependentJoint().has(frame);
    }

    std::vector<Joint*> getJointsFromFrames(const std::vector<Frame*>& frames)
    {
        std::vector<Joint*> active;

        typedef std::vector<Frame*>::const_iterator I;
        for (I p = frames.begin(); p != frames.end(); ++p) {
            Frame* frame = *p;

            // But how do we know that isActiveJoint() implies Joint*? Why don't
            // we use a dynamic cast here for safety?
            if (isActiveJoint(*frame) || isDependentJoint(*frame))
                active.push_back(static_cast<Joint*>(frame));
        }
        return active;
    }

    // From the root 'first' to the child 'last' inclusive.
    std::vector<Frame*> getChain(Frame* first, Frame* last, const State& state)
    {
        std::vector<Frame*> init = Kinematics::parentToChildChain(first, last, state);

        init.push_back(last);
        return init;
    }
}

SerialDevice::SerialDevice(Frame* first,
                           Frame* last,
                           const std::string& name,
                           const State& state):
    JointDevice(name, first, last,getJointsFromFrames(getChain(first, last, state)), state),
    _kinematicChain(getChain(first, last, state))
{
}

const std::vector<Frame*>& SerialDevice::frames() const
{
    return _kinematicChain;
}

SerialDevice::SerialDevice(const std::vector<Frame*>& serialChain,
                           const std::string& name, const State& state) :
    JointDevice(name, serialChain.front(), serialChain.back(),
                getJointsFromFrames(serialChain), state),

    _kinematicChain(serialChain)
{
}


