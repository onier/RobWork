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

#ifndef RW_GEOMETRY_ANALYTIC_ANALYTICUTIL_HPP_
#define RW_GEOMETRY_ANALYTIC_ANALYTICUTIL_HPP_

/**
 * @file AnalyticUtil.hpp
 *
 * \copydoc rw::geometry::AnalyticUtil
 */

#include <rw/math/Vector3D.hpp>

#include <list>
#include <vector>

namespace rw {
namespace geometry {

    class QuadraticCurve;

    //! @addtogroup geometry

    //! @{
    /**
     * @brief Utility functions for functions dealing with analytic geometry.
     */
    class AnalyticUtil
    {
        public:
            /**
             * @brief Combine discretized borders with curves to form polygons.
             *
             * The curves are themselves discretized to form the polygon.
             *
             * @param border [in] the full list of points.
             * @param subborder [in] a list of border sections.
             * Each section is a vector of indices into \b border.
             * @param curves [in] the curves to combine with the border
             * sections.
             * @param stepsPerRevolution [in] the resolution for discretization
             * of the curves.
             * @return a list of polygons. Each polygon is a list of points.
             */
            static std::list<std::vector<rw::math::Vector3D<> > > combinePolygons(
                    const std::vector<rw::math::Vector3D<> >& border,
                    const std::list<std::vector<std::size_t> >& subborder,
                    const std::vector<rw::geometry::QuadraticCurve>& curves,
                    double stepsPerRevolution);

        private:
            AnalyticUtil();
            virtual ~AnalyticUtil();
    };
//! @}

} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_ANALYTIC_ANALYTICUTIL_HPP_ */
