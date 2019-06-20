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

#include "ImplicitBREP.hpp"
#include "ImplicitFace.hpp"
#include "ImplicitShell.hpp"

using rw::common::ownedPtr;
using namespace rw::geometry;

namespace {
    // Shell Proxy class
    class BREPImplicitShell: public ImplicitShell {
        public:
            BREPImplicitShell(const ImplicitBREP* brep, double resolution): ImplicitShell(), _brep(brep) {
                _resolution = resolution;
            }
            ~BREPImplicitShell() {}
            bool isConvex() { return _brep->isConvex(); }
            std::size_t size() const { return _brep->faceCount(); }
            ImplicitFace::CPtr getFace(std::size_t idx) const {
                const ImplicitFace::Ptr bface = ownedPtr(new ImplicitFace(_brep->getSurface(idx).clone(), _brep->getVertices(idx)));
                const std::vector<ParametricCurve::Ptr> curves = _brep->getCurves(idx);
                for (std::size_t i = 0; i < curves.size(); i++) {
                    bface->setCurve(i,curves[i]);
                }
                bface->setMeshResolution(_resolution);
                return bface;
            }
            void getFace(std::size_t idx, ImplicitFace& dst) const {
                dst.setMeshResolution(_resolution);
                dst.setSurface(_brep->getSurface(idx));
                dst.setVertices(_brep->getVertices(idx));
                const std::vector<ParametricCurve::Ptr> curves = _brep->getCurves(idx);
                for (std::size_t i = 0; i < curves.size(); i++) {
                    dst.setCurve(i,curves[i]);
                }
            }

        private:
            Face::CPtr doGetFace(std::size_t idx) const {
                return getFace(idx);
            }

            const ImplicitBREP* const _brep;
    };
}

ImplicitBREP::ImplicitBREP():
    BREP()
{
}

ImplicitBREP::~ImplicitBREP()
{
}

GeometryData::GeometryType ImplicitBREP::getType() const
{
    return GeometryData::Implicit;
}

const ImplicitSurface& ImplicitBREP::getSurface(std::size_t surfaceIndex) const
{
    return *_surfaces[surfaceIndex];
}

const ParametricCurve& ImplicitBREP::getCurve(std::size_t curveIndex) const
{
    return *_curves[curveIndex];
}

void ImplicitBREP::scale(double factor)
{
    for (std::size_t i = 0; i < _vertices.size(); i++) {
        _vertices[i]->point *= factor;
    }
    for (std::size_t i = 0; i < _surfaces.size(); i++) {
        _surfaces[i] = _surfaces[i]->scale(factor);
    }
    for (std::size_t i = 0; i < _curves.size(); i++) {
        _curves[i] = _curves[i]->scale(factor);
    }
}

ImplicitBREP::Ptr ImplicitBREP::clone() const
{
    const ImplicitBREP::Ptr brep = ownedPtr(new ImplicitBREP());
    copyTopologyTo(brep);
    brep->_surfaces.resize(_surfaces.size());
    brep->_curves.resize(_curves.size());
    for (std::size_t i = 0; i < _surfaces.size(); i++) {
        brep->_surfaces[i] = _surfaces[i]->clone();
    }
    for (std::size_t i = 0; i < _curves.size(); i++) {
        brep->_curves[i] = _curves[i]->clone();
    }
    return brep;
}

ImplicitShell::CPtr ImplicitBREP::shellProxy() const
{
    return ownedPtr(new BREPImplicitShell(this,_resolution));
}

Shell::CPtr ImplicitBREP::doShellProxyBREP() const
{
    return ownedPtr(new BREPImplicitShell(this,_resolution));
}

std::vector<ParametricCurve::Ptr> ImplicitBREP::getCurves(std::size_t loopIdx) const
{
    return std::vector<ParametricCurve::Ptr>();
}

class ImplicitBREP::CommonParametricCurveSetImpl: public CommonParametricCurveSet {
public:
    CommonParametricCurveSetImpl(const ImplicitBREP* brep, const std::vector<const BREP::HalfEdge*>& edges): CommonParametricCurveSet(), _brep(brep), _edges(edges) {}
    ~CommonParametricCurveSetImpl() {}
    virtual std::size_t size() const { return _edges.size(); }
    virtual const ParametricCurve& curve(std::size_t index) const { return _brep->getCurve(_edges[index]->curveIndex); }
    virtual const ImplicitSurface& surfaceLeft(std::size_t index) const {
        return _brep->getSurface(_edges[index]->face->surfaceIndex);
    }
    virtual const ImplicitSurface& surfaceRight(std::size_t index) const {
        return _brep->getSurface(_edges[index]->oppositeEdge->face->surfaceIndex);
    }
private:
    const ImplicitBREP* const _brep;
    const std::vector<const BREP::HalfEdge*> _edges;
};

ImplicitBREP::CommonParametricCurveSet::CPtr ImplicitBREP::getCommonCurves(const std::set<std::size_t>& faces) const
{
    std::vector<const HalfEdge*> edges;
    for (std::set<std::size_t>::const_iterator it = faces.begin(); it != faces.end(); it++) {
        const HalfEdge* edge = _faces[*it]->edge;
        if (edge == NULL)
            continue;
        do {
            std::set<std::size_t>::const_iterator itB = it;
            for (itB++; itB != faces.end(); itB++) {
                const HalfEdge* edgeB = _faces[*itB]->edge;
                if (edgeB == NULL)
                    continue;
                do {
                    if (edge->oppositeEdge == edgeB) {
                        //curves.push_back(edge->curve);
                        edges.push_back(edge);
                    }
                    edgeB = edgeB->nextEdge;
                } while (edgeB != NULL && edgeB != _faces[*itB]->edge);
            }
            edge = edge->nextEdge;
        } while (edge != NULL && edge != _faces[*it]->edge);
    }
    return ownedPtr(new CommonParametricCurveSetImpl(this, edges));
}

void ImplicitBREP::addEdge(const ParametricCurve& curve, std::size_t v1, std::size_t v2)
{
    _curves.push_back(curve.clone());
    addBREPEdge(_curves.size()-1,v1,v2);
}

void ImplicitBREP::setFace(const ImplicitSurface& surface, std::size_t loop)
{
    std::size_t surfaceIndex;
    const ImplicitSurface::Ptr newSurface = surface.clone();
    if (hasSurfaceSet(loop)) {
        surfaceIndex = getSurfaceIndex(loop);
        _surfaces[surfaceIndex] = newSurface;
    } else {
        _surfaces.push_back(newSurface);
        surfaceIndex = _surfaces.size()-1;
    }
    for (std::size_t i = 0; i < _surfaces.size(); i++) {
        if (i != surfaceIndex)
            _surfaces[i]->reuseTrimmingRegions(newSurface);
    }
    setBREPFace(surfaceIndex, loop);
}

void ImplicitBREP::doRemoveCurve(const std::size_t curveIndex)
{
    _curves.erase(_curves.begin()+curveIndex);
}
