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

#include <rwlibs/opengl/DrawableUtil.hpp>

#include "RenderParametricSurface.hpp"
#include "ParametricPlaneFace.hpp"
#include "ParametricRoundedEdge.hpp"

using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::math;
using namespace rwlibs::nurbs;
using namespace rwlibs::opengl;

RenderParametricSurface::RenderParametricSurface(ParametricSurface::Ptr surface):
	_surface(surface),
	_r(0.8f),_g(0.8f),_b(0.8f)
{
}

RenderParametricSurface::~RenderParametricSurface() {
}

void RenderParametricSurface::setColor(float r, float g, float b) {
	_r = r;
	_g = g;
	_b = b;
}

void RenderParametricSurface::draw(const DrawableNode::RenderInfo& info,
    		DrawableNode::DrawType type,
    		double alpha) const {
	glPushMatrix();

	DrawableUtil::multGLTransform(Transform3D<>::identity());

	glColor4f(_r, _g, _b, (float)alpha);
	switch(type){
	case DrawableNode::SOLID:
		glPolygonMode(GL_FRONT, GL_FILL);
		break;
	case DrawableNode::OUTLINE: // Draw nice frame
		glPolygonMode(GL_FRONT, GL_FILL);
	case DrawableNode::WIRE: // Draw nice frame
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		break;
	}

	if (ParametricPlaneFace::Ptr plane = _surface.cast<ParametricPlaneFace>()) {
		renderRectangle();
	} else if (ParametricRoundedEdge::Ptr edge = _surface.cast<ParametricRoundedEdge>()) {
		renderTriangles();
	}
	glPopMatrix();
}

void RenderParametricSurface::renderTriangles() const {
	glBegin(GL_TRIANGLES);
	TriMesh::Ptr mesh = _surface->getTriMesh();
	for(size_t i=0;i<mesh->getSize();i++){
		Triangle<double> tri = mesh->getTriangle(i);
		Vector3D<float> n = cast<float>(tri.calcFaceNormal());
		Vector3D<float> v0 = cast<float>(tri[0]);
		Vector3D<float> v1 = cast<float>(tri[1]);
		Vector3D<float> v2 = cast<float>(tri[2]);
		glNormal3fv(&n[0]);
		glVertex3fv(&v0[0]);
		glVertex3fv(&v1[0]);
		glVertex3fv(&v2[0]);
	}
	glEnd();
}

void RenderParametricSurface::renderRectangle() const {
	ParametricPlaneFace::Ptr plane = _surface.cast<ParametricPlaneFace>();
    glBegin(GL_QUADS);
    Vector3D<float> n = cast<float>(plane->normal(0.,0.));
    glNormal3fv(&n[0]);

	std::vector<Vector3D<> > corners = plane->getCorners();
    for(std::vector<Vector3D<> >::iterator it = corners.begin(); it < corners.end(); it++){
    	Vector3D<float> vertex = cast<float>(*it);
        glVertex3fv(&vertex[0]);
    }
    glEnd();
}
