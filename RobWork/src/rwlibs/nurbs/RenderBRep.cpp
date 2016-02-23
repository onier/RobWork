/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "NURBSCurve.hpp"
#include "NURBSSurface.hpp"
#include "ParametricSurface.hpp"
#include "ParametricCurve.hpp"
#include "RenderBRep.hpp"
#include "RenderParametricCurve.hpp"
#include "RenderParametricSurface.hpp"
#include "RenderNURBSCurve.hpp"
#include "RenderNURBSSurface.hpp"

#include <rwlibs/opengl/RenderPointCloud.hpp>

using namespace rw::graphics;
using namespace rw::math;
using namespace rwlibs::nurbs;
using namespace rwlibs::opengl;

RenderBRep::RenderBRep(BRep::Ptr brep)
{
	{
		std::list<ParametricSurface::Ptr> faces = brep->getSurfaces();
		std::list<ParametricSurface::Ptr>::iterator it;
		for (it = faces.begin(); it != faces.end(); it++) {
			RenderParametricSurface* render;
			if (NURBSSurface::Ptr nurbs = (*it).cast<NURBSSurface>())
				render = new RenderNURBSSurface(nurbs);
			else
				render = new RenderParametricSurface(*it);
			_faceRenders.push_back(render);
		}
	}

	{
		std::list<ParametricCurve::Ptr> faces = brep->getCurves();
		std::list<ParametricCurve::Ptr>::iterator it;
		for (it = faces.begin(); it != faces.end(); it++) {
			RenderParametricCurve* render;
			if (NURBSCurve::Ptr nurbs = (*it).cast<NURBSCurve>())
				render = new RenderNURBSCurve(nurbs);
			else
				render = new RenderParametricCurve(*it);
			_edgeRenders.push_back(render);
		}
	}

	{
		RenderPointCloud* render = new RenderPointCloud();
		std::list<Vector3D<> > points = brep->getPoints();
		std::list<Vector3D<> >::iterator it;
		for (it = points.begin(); it != points.end(); it++) {
			render->addPoint(*it);
		}
		_vertexRenders.push_back(render);
	}
}

RenderBRep::~RenderBRep() {
	std::list<Render*>::iterator it;

	for (it = _faceRenders.begin(); it != _faceRenders.end(); it++)
		delete *it;

	for (it = _edgeRenders.begin(); it != _edgeRenders.end(); it++)
		delete *it;

	for (it = _vertexRenders.begin(); it != _vertexRenders.end(); it++)
		delete *it;

	_faceRenders.clear();
	_edgeRenders.clear();
	_vertexRenders.clear();
}

void RenderBRep::draw(const DrawableNode::RenderInfo& info,
			DrawableNode::DrawType type,
			double alpha) const
{
	std::list<Render*>::const_iterator it;

	for (it = _faceRenders.begin(); it != _faceRenders.end(); it++)
		(*it)->draw(info, type, alpha);

	for (it = _edgeRenders.begin(); it != _edgeRenders.end(); it++)
		(*it)->draw(info, type, alpha);

	for (it = _vertexRenders.begin(); it != _vertexRenders.end(); it++)
		(*it)->draw(info, type, alpha);
}
