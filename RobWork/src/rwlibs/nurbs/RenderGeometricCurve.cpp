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

#include "RenderGeometricCurve.hpp"

using namespace rw::graphics;
using namespace rw::math;
using namespace rwlibs::opengl;
using namespace rwlibs::nurbs;

RenderGeometricCurve::RenderGeometricCurve(const GeometricCurve* const curve):
	_lineRender(new RenderLines()),
	_curve(curve),
	_r(0.8f),_g(0.8f),_b(0.8f)
{
}

RenderGeometricCurve::~RenderGeometricCurve() {
	delete _lineRender;
}

void RenderGeometricCurve::setColor(float r, float g, float b) {
	_r = r;
	_g = g;
	_b = b;
}

void RenderGeometricCurve::draw(const DrawableNode::RenderInfo& info,
		DrawableNode::DrawType type,
		double alpha) const {
	const std::size_t SEGMENTS = 10;
	_lineRender->clear();
	for (std::size_t i = 0; i < SEGMENTS; i++) {
		double from = ((double)i)/((double)SEGMENTS);
		double to = ((double)(i+1))/((double)SEGMENTS);
		_lineRender->addLine(_curve->evaluate(from),_curve->evaluate(to));
	}
	_lineRender->draw(info,type,alpha);
}

Vector3D<float> RenderGeometricCurve::getColor() const {
	return Vector3D<float>(_r,_g,_b);
}
