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

#include "AnalyticUtil.hpp"

#include <rw/geometry/analytic/quadratics/QuadraticCurve.hpp>

using namespace rw::geometry;
using rw::math::Vector3D;

AnalyticUtil::AnalyticUtil()
{
}

AnalyticUtil::~AnalyticUtil()
{
}

std::list<std::vector<Vector3D<> > > AnalyticUtil::combinePolygons(
        const std::vector<Vector3D<> >& border,
        const std::list<std::vector<std::size_t> >& subborder,
        const std::vector<QuadraticCurve>& curves,
        const double stepsPerRevolution)
{
    std::list<std::vector<std::size_t> >::const_iterator curPol;
    // Try to connect polygons if open
    std::list<std::vector<Vector3D<> > > fullPolygons;
    for (curPol = subborder.begin(); curPol != subborder.end(); curPol++) {
        fullPolygons.resize(fullPolygons.size()+1);
        std::vector<Vector3D<> >& fullPolygon = fullPolygons.back();
        const QuadraticCurve* closest = &curves[0];
        const Vector3D<>& P1 = border[curPol->front()];
        const Vector3D<>& P2 = border[curPol->back()];
        double time1 = curves[0].closestTime(P1);
        double time2 = curves[0].closestTime(P2);
        double dist = std::min((curves[0](time1)-P1).norm2(),((curves[0](time2)-P2).norm2()));
        for (std::size_t i = 1; i < curves.size(); i++) {
            time1 = curves[i].closestTime(P1);
            time2 = curves[i].closestTime(P2);
            const double d = std::min((curves[i](time1)-P1).norm2(),((curves[i](time2)-P2).norm2()));
            if (d < dist) {
                closest = &curves[i];
                dist = d;
            }
        }
        time1 = closest->closestTime(P1);
        time2 = closest->closestTime(P2);
        QuadraticCurve cp(*closest);
        cp.setLimits(std::make_pair((time1<time2)?time1:time2,(time1<time2)?time2:time1));
        for (std::size_t k = 0; k < curPol->size(); k++) {
            fullPolygon.push_back(border[(*curPol)[k]]);
        }

        const std::list<Vector3D<> > seg = cp.discretizeAdaptive(stepsPerRevolution);
        if (seg.size() > 1) {
            if ((seg.front()-P1).norm2() < (seg.front()-P2).norm2()) {
                fullPolygon.insert(fullPolygon.end(),++seg.rbegin(),--seg.rend());
            } else {
                fullPolygon.insert(fullPolygon.end(),++seg.begin(),--seg.end());
            }
        }
    }
    return fullPolygons;
}
