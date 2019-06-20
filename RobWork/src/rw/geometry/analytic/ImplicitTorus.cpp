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

#include "ImplicitTorus.hpp"

#include <rw/geometry/Polygon.hpp>
#include <rw/geometry/PolygonUtil.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/geometry/Delaunay.hpp>
#include <rw/geometry/analytic/AnalyticUtil.hpp>
#include <rw/geometry/analytic/quadratics/QuadraticCurve.hpp>
#include <rw/geometry/analytic/quadratics/QuadraticSurface.hpp>
#include <rw/math/Vector2D.hpp>

using rw::common::ownedPtr;
using namespace rw::geometry;
using namespace rw::math;

ImplicitTorus::ImplicitTorus(double R, double r):
    _R1(R),
    _R2(R),
    _r1(r),
    _r2(r),
    _stepsPerRevolution(10),
    _isNormalForm(true)
{
}

ImplicitTorus::ImplicitTorus(double R1, double R2, double r1, double r2):
    _R1(R1),
    _R2(R2),
    _r1(r1),
    _r2(r2),
    _stepsPerRevolution(10),
    _isNormalForm(true)
{
}

ImplicitTorus::ImplicitTorus(double R1, double R2, double r1, double r2,
        const Transform3D<>& transform,
        const std::vector<TrimmingRegion>& conditions, double stepsPerRevolution):
    _R1(R1),
    _R2(R2),
    _r1(r1),
    _r2(r2),
    _conditions(conditions),
    _stepsPerRevolution(stepsPerRevolution),
    _transform(transform),
    _isNormalForm(false)
{
}

ImplicitTorus::~ImplicitTorus()
{
}

ImplicitTorus::Ptr ImplicitTorus::transform(const Transform3D<>& T) const
{
    std::vector<TrimmingRegion> conditions;
    for (std::size_t i = 0; i < _conditions.size(); i++)
        conditions.push_back(_conditions[i]->transform(T));
    return ownedPtr(new ImplicitTorus(_R1,_R2,_r1,_r2,_transform*T, conditions,
            _stepsPerRevolution));
}

ImplicitTorus::Ptr ImplicitTorus::transform(const Vector3D<>& P) const
{
    Transform3D<> T = _transform;
    T.P() += P;
    std::vector<TrimmingRegion> conditions;
    for (std::size_t i = 0; i < _conditions.size(); i++)
        conditions.push_back(_conditions[i]->transform(T));
    return ownedPtr(new ImplicitTorus(_R1,_R2,_r1,_r2,T, conditions,
            _stepsPerRevolution));
}

ImplicitTorus::Ptr ImplicitTorus::scale(double factor) const
{
    std::vector<TrimmingRegion> conditions;
    for (std::size_t i = 0; i < _conditions.size(); i++) {
        conditions.push_back(_conditions[i]->scale(factor));
    }
    return ownedPtr(new ImplicitTorus(_R1*factor, _R2*factor,
            _r1*factor, _r2*factor, _transform, conditions,
            _stepsPerRevolution));
}

ImplicitTorus::Ptr ImplicitTorus::clone() const
{
    std::vector<TrimmingRegion> conditions;
    for (std::size_t i = 0; i < _conditions.size(); i++) {
        conditions.push_back(_conditions[i]->clone());
    }
    return ownedPtr(new ImplicitTorus(_R1,_R2,_r1,_r2,_transform,conditions,
            _stepsPerRevolution));
}

std::pair<double,double> ImplicitTorus::extremums(const Vector3D<>& direction) const
{
    std::pair<double,double> extremums;
    const double maxPlane = std::max(_R1+_r1,_R2+_r1);
    extremums.first = -std::sqrt(maxPlane*maxPlane+_r2*_r2);
    extremums.second = -extremums.first;
    // todo: find actual ranges based on the direction argument
    return extremums;
}

TriMesh::Ptr ImplicitTorus::getTriMesh(const std::vector<Vector3D<> >& border) const
{
    if (_isNormalForm)
        return getTriMeshNormalForm(border);

    const Transform3D<> Tinv = inverse(_transform);
    std::vector<Vector3D<> > borderRot(border.size());
    for (std::size_t i = 0; i < border.size(); i++)
        borderRot[i] = Tinv*border[i];
    std::vector<TrimmingRegion> conditions;
    for (std::size_t i = 0; i < _conditions.size(); i++)
        conditions.push_back(_conditions[i]->transform(Tinv));
    const ImplicitTorus normalForm(_R1,_R2,_r1,_r2,_transform,conditions,
            _stepsPerRevolution);
    return normalForm.getTriMeshNormalForm(borderRot, _transform);
}

TriMesh::Ptr ImplicitTorus::getTriMeshNormalForm(const std::vector<Vector3D<> >& border, const Transform3D<>& transform) const
{
    static const double EPS = std::numeric_limits<double>::epsilon()*5;

    if (_R1 != _R2)
        RW_THROW("ImplicitTorus can not yet convert to trimesh when R1 != R2");

    const PlainTriMeshN1D::Ptr mesh = ownedPtr(new PlainTriMeshN1D());

    // Split border polygon in front and back (when seen in uv plane)
    std::list<std::vector<std::size_t> > borderFront;
    std::list<std::vector<std::size_t> > borderBack;
    // Front
    {
        Vector3D<> last = border.front();
        std::vector<std::size_t> cur;
        if (last[2] >= -EPS)
            cur.push_back(0);
        for (std::size_t i = 1; i < border.size(); i++) {
            if (last[2] >= -EPS && border[i][2] >= -EPS) {
                cur.push_back(i);
            } else if (last[2] < -EPS) {
                if (cur.size() > 0) {
                    borderFront.push_back(cur);
                    cur.clear();
                }
                if (border[i][2] >= -EPS)
                    cur.push_back(i);
            }
            last = border[i];
        }
        if (last[2] >= -EPS) {
            if (border.front()[2] >= -EPS && borderFront.size() > 0) {
                borderFront.front().insert(borderFront.front().begin(),cur.begin(),cur.end());
                cur.clear();
            }
        }
        if (cur.size() > 0) {
            borderFront.push_back(cur);
        }
    }
    // Back
    {
        Vector3D<> last = border.front();
        std::vector<std::size_t> cur;
        if (last[2] <= EPS)
            cur.push_back(0);
        for (std::size_t i = 1; i < border.size(); i++) {
            if (last[2] <= EPS && border[i][2] <= EPS) {
                cur.push_back(i);
            } else if (last[2] > EPS) {
                if (cur.size() > 0) {
                    borderBack.push_back(cur);
                    cur.clear();
                }
                if (border[i][2] <= EPS)
                    cur.push_back(i);
            }
            last = border[i];
        }
        if (last[2] <= EPS) {
            if (border.front()[2] <= EPS && borderBack.size() > 0) {
                borderBack.front().insert(borderBack.front().begin(),cur.begin(),cur.end());
                cur.clear();
            }
        }
        if (cur.size() > 0) {
            borderBack.push_back(cur);
        }
    }

    // If there are front border segments lying completely inside the +/- EPS region, remove them if they are part of a larger back segment.
    {
        std::list<std::vector<std::size_t> >& listA = borderFront;
        const std::list<std::vector<std::size_t> >& listB = borderBack;

        for (std::list<std::vector<std::size_t> >::iterator it = listA.begin(); it != listA.end();) { // do not use const_iterator (first supported with list.erase() from GCC 4.9)
            bool within = true;
            for (std::size_t i = 0; i < (*it).size() && within; i++) {
                if (std::abs(border[(*it)[i]][2]) > EPS) {
                    within = false;
                }
            }
            if (within) {
                bool match = false;
                for (std::list<std::vector<std::size_t> >::const_iterator itB = listB.begin(); itB != listB.end() && !match; itB++) {
                    bool outsideB = false;
                    for (std::size_t i = 0; i < (*itB).size() && !outsideB; i++) {
                        if (std::abs(border[(*itB)[i]][2]) > EPS) {
                            outsideB = true;
                        }
                    }
                    if (outsideB) {
                        bool firstMatched = false;
                        std::size_t i = 0;
                        for (std::size_t j = 0; j <= (*itB).size() && !match; j++) {
                            if (j == (*itB).size()) {
                                if (firstMatched)
                                    j = 0;
                                else
                                    break;
                            }
                            if (!firstMatched) {
                                if ((*itB)[j] == (*it)[i]) {
                                    firstMatched = true;
                                    i++;
                                }
                            } else {
                                if (i >= (*it).size()) {
                                    match = true;
                                } else if ((*itB)[j] != (*it)[i]) {
                                    match = false;
                                    break;
                                } else if ((*itB)[j] == (*it)[i]) {
                                    i++;
                                }
                            }
                        }
                    }
                }
                if (match) {
                    it = listA.erase(it);
                } else {
                    it++;
                }
            } else {
                it++;
            }
        }
    }
    // If there are back border segments lying completely inside the +/- EPS region, remove them if they are part of a larger front segment.
    {
        std::list<std::vector<std::size_t> >& listA = borderBack;
        const std::list<std::vector<std::size_t> >& listB = borderFront;

        for (std::list<std::vector<std::size_t> >::iterator it = listA.begin(); it != listA.end();) { // do not use const_iterator (first supported with list.erase() from GCC 4.9)
            bool within = true;
            for (std::size_t i = 0; i < (*it).size() && within; i++) {
                if (std::abs(border[(*it)[i]][2]) > EPS) {
                    within = false;
                }
            }
            if (within) {
                bool match = false;
                for (std::list<std::vector<std::size_t> >::const_iterator itB = listB.begin(); itB != listB.end() && !match; itB++) {
                    bool outsideB = false;
                    for (std::size_t i = 0; i < (*itB).size() && !outsideB; i++) {
                        if (std::abs(border[(*itB)[i]][2]) > EPS) {
                            outsideB = true;
                        }
                    }
                    if (outsideB) {
                        bool firstMatched = false;
                        std::size_t i = 0;
                        for (std::size_t j = 0; j <= (*itB).size() && !match; j++) {
                            if (j == (*itB).size()) {
                                if (firstMatched)
                                    j = 0;
                                else
                                    break;
                            }
                            if (!firstMatched) {
                                if ((*itB)[j] == (*it)[i]) {
                                    firstMatched = true;
                                    i++;
                                }
                            } else {
                                if (i >= (*it).size()) {
                                    match = true;
                                } else if ((*itB)[j] != (*it)[i]) {
                                    match = false;
                                    break;
                                } else if ((*itB)[j] == (*it)[i]) {
                                    i++;
                                }
                            }
                        }
                    }
                }
                if (match) {
                    it = listA.erase(it);
                } else {
                    it++;
                }
            } else {
                it++;
            }
        }
    }

    // Find silhouette of surface
    std::vector<QuadraticCurve> silhouette;
    silhouette.push_back(QuadraticCurve(Vector3D<>::zero(), Vector3D<>::x()*(_R1-_r1),-Vector3D<>::y()*(_R2-_r1), QuadraticCurve::Elliptic));
    silhouette.push_back(QuadraticCurve(Vector3D<>::zero(), Vector3D<>::x()*(_R1+_r1), Vector3D<>::y()*(_R2+_r1), QuadraticCurve::Elliptic));

    // Try to connect polygons if open
    const std::list<std::vector<Vector3D<> > > fullPolygonFront = AnalyticUtil::combinePolygons(border, borderFront, silhouette, _stepsPerRevolution);
    const std::list<std::vector<Vector3D<> > > fullPolygonBack = AnalyticUtil::combinePolygons(border, borderBack, silhouette, _stepsPerRevolution);

    for (std::list<std::vector<Vector3D<> > >::const_iterator it = fullPolygonFront.begin(); it != fullPolygonFront.end(); it++) {
        makeSurface(*it, FRONT, mesh);
    }
    for (std::list<std::vector<Vector3D<> > >::const_iterator it = fullPolygonBack.begin(); it != fullPolygonBack.end(); it++) {
        makeSurface(*it, BACK, mesh);
    }

    return mesh;
}

void ImplicitTorus::makeSurface(const std::vector<Vector3D<> > fullPolygon, Place place, PlainTriMeshN1D::Ptr mesh) const {
    // Project border
    Polygon<Vector2D<> > polygon;
    for (std::size_t i = 0; i < fullPolygon.size(); i++) {
        polygon.addVertex(Vector2D<>(fullPolygon[i][0],fullPolygon[i][1]));
    }

    // Do convex decomposition
    if (polygon.size() < 3)
        RW_THROW("Can not decompose polygon that has " << polygon.size() << " vertices.");
    const std::vector<std::vector<std::size_t> > convexPolygonsIndexed = PolygonUtil::convexDecompositionIndexed(polygon);

    std::vector<Polygon<Vector2D<> > > convexPolygons(convexPolygonsIndexed.size());
    {
        // Construct plain polygon
        std::size_t i = 0;
        for (std::vector<std::vector<std::size_t> >::const_iterator it = convexPolygonsIndexed.begin(); it != convexPolygonsIndexed.end(); it++) {
            const std::vector<std::size_t>& subPoly = *it;
            for (std::size_t k = 0; k < subPoly.size(); k++) {
                convexPolygons[i].addVertex(polygon[subPoly[k]]);
                // Check if edge is shared with other polygon
                const std::size_t idxNext = subPoly[(k+1)%subPoly.size()];
                bool inOther = false;
                for (std::vector<std::vector<std::size_t> >::const_iterator itB = convexPolygonsIndexed.begin(); itB != convexPolygonsIndexed.end() && inOther == false; itB++) {
                    if (itB == it)
                        continue;
                    const std::vector<std::size_t>& subPolyB = *itB;
                    for (std::size_t kB = 0; kB < subPolyB.size() && inOther == false; kB++) {
                        const std::size_t idxNextB = subPolyB[(kB+1)%subPolyB.size()];
                        if (subPolyB[kB] == subPoly[k] && idxNextB == idxNext)
                            inOther = true;
                        else if (subPolyB[kB] == idxNext && idxNextB == subPoly[k])
                            inOther = true;
                    }
                }
                if (inOther) {
                    // Sample inner border
                    const Vector2D<> dp = polygon[idxNext]-polygon[subPoly[k]];
                    std::size_t samples = 2;
                    for (std::size_t sI = 1; sI < samples; sI++) {
                        convexPolygons[i].addVertex(polygon[subPoly[k]]+dp*((double)sI/samples));
                    }
                }
            }
            i++;
        }
    }

    // Sample surface points
    double minU = convexPolygons[0][0][0];
    double maxU = convexPolygons[0][0][0];
    double minV = convexPolygons[0][0][1];
    double maxV = convexPolygons[0][0][1];
    for (std::size_t i = 0; i < convexPolygons.size(); i++) {
        const Polygon<Vector2D<> >& convexPoly = convexPolygons[i];
        for (std::size_t k = 0; k < convexPoly.size(); k++) {
            if (convexPoly[k][0] < minU)
                minU = convexPoly[k][0];
            if (convexPoly[k][0] > maxU)
                maxU = convexPoly[k][0];
            if (convexPoly[k][1] < minV)
                minV = convexPoly[k][1];
            if (convexPoly[k][1] > maxV)
                maxV = convexPoly[k][1];
        }
    }

    static const std::size_t nrPoints = 2;
    static const std::size_t nrPointsU = nrPoints;
    static const std::size_t nrPointsV = nrPoints;
    const double dU = (maxU-minU)/nrPointsU;
    const double dV = (maxV-minV)/nrPointsV;
    std::vector<Vector2D<> > surfacePoints((nrPointsU-1)*(nrPointsV-1));
    for (std::size_t ui = 1; ui < nrPointsU; ui++) {
        const double u = minU+dU*ui;
        for (std::size_t vi = 1; vi < nrPointsV; vi++) {
            const double v = minV+dV*vi;
            surfacePoints[(ui-1)*(nrPointsV-1)+vi-1][0] = u;
            surfacePoints[(ui-1)*(nrPointsV-1)+vi-1][1] = v;
        }
    }

    for (std::size_t i = 0; i < convexPolygons.size(); i++) {
        const Polygon<Vector2D<> >& convexPoly = convexPolygons[i];

        std::vector<Vector2D<> > points2d(convexPoly.size());
        for (std::size_t pi = 0; pi < convexPoly.size(); pi++) {
            points2d[pi] = convexPoly[pi];
        }

        // Take the surface points in the convex region
        for (std::size_t k = 0; k < surfacePoints.size(); k++) {
            if (PolygonUtil::isInsideConvex(surfacePoints[k],convexPoly,1e-1)) {
                points2d.push_back(surfacePoints[k]);
            }
        }

        // Triangulate with Delaunay
        const IndexedTriMesh<>::Ptr meshProj = Delaunay::triangulate(points2d);

        // Unproject
        for (std::size_t triI = 0; triI < meshProj->size(); triI++) {
            Triangle<double> tri = meshProj->getTriangle(triI);
            bool br = false;
            for (std::size_t pi = 0; pi < 3; pi++) {
                const double uVal = tri[pi][0];
                const double vVal = tri[pi][1];
                double eVal;
                if (_R1 == _R2) {
                    const double uvSq = uVal*uVal+vVal*vVal;
                    const double f = _r2*_r2/(_r1*_r1);
                    eVal = _r2*_r2-f*(_R2*_R2+uvSq-2.*_R2*std::sqrt(uvSq));
                    eVal = (eVal >= 0) ? std::sqrt(eVal) : 0;
                } else {
                    RW_THROW("R1 != R2 is not yet implemented!");
                }
                if ((place == FRONT && eVal < 0) || (place == BACK && eVal > 0)) {
                    br = true;
                    break;
                }
                tri[pi] = _transform*Vector3D<>(uVal,vVal,eVal);
            }
            if (br)
                continue;
            const double nu = 0;
            const double nv = 0;
            const double ne = 1;
            const Vector3D<> normal = _transform*Vector3D<>(nu, nv, ne);
            if (dot(tri.calcFaceNormal(),normal) < 0) {
                mesh->add(TriangleN1<>(tri[0],tri[2],tri[1],normal));
            } else {
                mesh->add(TriangleN1<>(tri[0],tri[1],tri[2],normal));
            }
        }
    }
}

bool ImplicitTorus::equals(const Surface& surface, double threshold) const
{
    const ImplicitTorus* const qsurf = dynamic_cast<const ImplicitTorus*>(&surface);
    if (qsurf != nullptr) {
        // todo: recognize rotated toruses as identical surfaces.
        if (std::fabs(_R1-qsurf->_R1) > threshold)
            return false;
        if (std::fabs(_R2-qsurf->_R2) > threshold)
            return false;
        if (std::fabs(_r1-qsurf->_r1) > threshold)
            return false;
        if (std::fabs(_r2-qsurf->_r2) > threshold)
            return false;
        if (!_transform.equal(qsurf->_transform,threshold))
            return false;
        std::vector<bool> omatched(qsurf->_conditions.size(), false);
        for (const QuadraticSurface::TrimmingRegion treg : _conditions) {
            bool tmatched = false;
            for (std::size_t i = 0; i < qsurf->_conditions.size(); i++) {
                if (treg->equals(*qsurf->_conditions[i],threshold)) {
                    tmatched = true;
                    omatched[i] = true;
                }
            }
            if (!tmatched)
                return false;
        }
        for (const bool match : omatched) {
            if (!match)
                return false;
        }
        return true;
    } else {
        return false;
    }
}

double ImplicitTorus::operator()(const Vector3D<>& x) const
{
    const double r1sq = _r1*_r1;
    double rTerm;
    if (_r1 == _r2)
        rTerm = r1sq-x[2]*x[2];
    else
        rTerm = r1sq*(1.-x[2]*x[2]/(_r2*_r2));
    if (_R1 == _R2) {
        const double Rsq = _R1*_R2;
        return std::pow(x[0]*x[0]+x[1]*x[1]-Rsq-rTerm, 2.)-4.*Rsq*rTerm;
    } else {
        const double R1sq = _R1*_R1;
        const double R2sq = _R2*_R2;
        const double R1R2 = _R1*_R2;
        const double x0sq = x[0]*x[0];
        const double x1sq = x[1]*x[1];
        const double l = (R2sq + rTerm)*x0sq + (R1sq + rTerm)*x1sq - (R1sq + rTerm)*(R2sq + rTerm) - 4.*R1R2*rTerm;
        const double r = _R2*x0sq + _R1*x1sq - (_R1+_R2)*(R1R2 + rTerm);
        return std::pow(l, 2.) - 4.*rTerm*std::pow(r, 2.);
    }
}

bool ImplicitTorus::insideTrimmingRegion(const Vector3D<>& P) const
{
    for (std::size_t i = 0; i < _conditions.size(); i++) {
        if ((*_conditions[i])(P) > 0)
            return false;
    }
    return true;
}

Vector3D<> ImplicitTorus::normal(const Vector3D<>& x) const
{
    return normalize(gradient(x));
}

Vector3D<> ImplicitTorus::gradient(const Vector3D<>& x) const
{
    const double r1sq = _r1*_r1;
    const double r2sq = _r2*_r2;
    double rTerm;
    if (_r1 == _r2)
        rTerm = r1sq-x[2]*x[2];
    else
        rTerm = r1sq*(1.-x[2]*x[2]/(_r2*_r2));
    if (_R1 == _R2) {
        const double Rsq = _R1*_R2;
        Vector3D<> g = 4.*(x[0]*x[0]+x[1]*x[1]-Rsq-rTerm)*x;
        g[2] += 8.*Rsq*x[2];
        if (_r1 != _r2)
            g[2] *= r2sq/r1sq;
        return g;
    } else {
        const double R1sq = _R1*_R1;
        const double R2sq = _R2*_R2;
        const double R1R2 = _R1*_R2;
        const double x0sq = x[0]*x[0];
        const double x1sq = x[1]*x[1];
        const double l = (R2sq + rTerm)*x0sq + (R1sq + rTerm)*x1sq - (R1sq + rTerm)*(R2sq + rTerm) - 4.*R1R2*rTerm;
        const double r = _R2*x0sq + _R1*x1sq - (_R1+_R2)*(R1R2 + rTerm);
        double drTerm;
        if (_r1 == _r2)
            drTerm = -2.*x[2];
        else
            drTerm = -2.*r1sq/r2sq*x[2];
        Vector3D<> g;
        g[0] = 4.*l*(R2sq+rTerm)*x[0]-16.*rTerm*r*_R2*x[0];
        g[1] = 4.*l*(R1sq+rTerm)*x[1]-16.*rTerm*r*_R1*x[1];
        g[2] = 2.*l*drTerm*(x0sq+x1sq-(R1sq+R2sq)-2.*rTerm-4.*R1R2) - 4.*r*r*drTerm + 8.*r*(_R1+_R2)*rTerm*drTerm;
        return g;
    }
}

void ImplicitTorus::reuseTrimmingRegions(const ImplicitSurface::Ptr surface) const
{
    const ImplicitTorus::Ptr tsurf = surface.cast<ImplicitTorus>();
    if (!tsurf.isNull()) {
        for (std::size_t i = 0; i < tsurf->_conditions.size(); i++) {
            const ImplicitTorus::TrimmingRegion reg = tsurf->_conditions[i];
            for (const ImplicitTorus::TrimmingRegion treg : _conditions) {
                if (reg->equals(*treg,1e-15)) {
                    tsurf->_conditions[i] = treg;
                    break;
                }
            }
        }
    }
    const QuadraticSurface::Ptr qsurf = surface.cast<QuadraticSurface>();
    if (!qsurf.isNull()) {
        std::vector<QuadraticSurface::TrimmingRegion> regs = qsurf->getTrimmingConditions();
        for (std::size_t i = 0; i < regs.size(); i++) {
            const ImplicitTorus::TrimmingRegion reg = regs[i];
            for (const ImplicitTorus::TrimmingRegion treg : _conditions) {
                if (reg->equals(*treg,1e-15)) {
                    regs[i] = treg;
                    break;
                }
            }
        }
        qsurf->setTrimmingConditions(regs);
    }
}
