/******************************************************************************
 * Copyright 2019 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
 ******************************************************************************/

#ifndef RWLIBS_GEOMETRY_ANALYTIC_GEOMETRY_QUADRATICTESTOBJECTS_HPP_
#define RWLIBS_GEOMETRY_ANALYTIC_GEOMETRY_QUADRATICTESTOBJECTS_HPP_

/**
 * @file QuadraticTestObjects.hpp
 *
 * \copydoc rwlibs::geometry::QuadraticTestObjects
 */

#include <rw/common/Ptr.hpp>

namespace rw { namespace geometry { class QuadraticBREP; } }

namespace rwlibs {
    namespace geometry {

        //! @addtogroup rwlibs_geometry

        //! @{
        /**
         * @brief Library of objects that are made up of Quadratic surfaces.
         */
        class QuadraticTestObjects
        {
            public:
                /**
                 * @brief
                 * @return
                 */
                static rw::common::Ptr<rw::geometry::QuadraticBREP> objectA();

                /**
                 * @brief
                 * @return
                 */
                static rw::common::Ptr<rw::geometry::QuadraticBREP> objectB();

            private:
                QuadraticTestObjects();
                virtual ~QuadraticTestObjects();
        };
    //! @}

    } /* namespace geometry */
} /* namespace rwlibs */

#endif /* RWLIBS_GEOMETRY_ANALYTIC_GEOMETRY_QUADRATICTESTOBJECTS_HPP_ */
