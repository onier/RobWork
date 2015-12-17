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

#include <boost/foreach.hpp>
#include "RWPEBodyConstraintGraph.hpp"
#include "RWPEDebugRender.hpp"
#include "RWPEBodyDynamic.hpp"
#include "RWPEContact.hpp"

using namespace rw::graphics;
using namespace rw::math;
using namespace rwlibs::opengl;
using namespace rwsimlibs::rwpe;

RWPEDebugRender::RWPEDebugRender():
	_drawMask(0),
	_manager(NULL),
	_state(NULL),
	_gravity(Vector3D<>::zero())
{
}

RWPEDebugRender::~RWPEDebugRender() {
}

void RWPEDebugRender::setDrawMask(unsigned int mask) {
	_drawMask = mask;
}

void RWPEDebugRender::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const {
	if (_manager == NULL || _state == NULL)
		return;

    if(DRAW_CONTACT_NORMAL & _drawMask){
        glPushMatrix();
        const RWPEBodyConstraintGraph::ConstraintList constraints = _manager->getTemporaryConstraints(_state);
        BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
        	const RWPEContact* const contact = dynamic_cast<const RWPEContact*>(constraint);
        	if (contact) {
                const Vector3D<>& pParent = contact->getPositionParentW(*_state);
                glLineWidth((GLfloat)(1.0f));
                glBegin(GL_LINES);
                glColor3f(1.0, 0.0, 0.0);
                DrawableUtil::drawGLVertex(pParent);
                glColor3f(0.0, 1.0, 0.0);
                DrawableUtil::drawGLVertex(pParent + contact->getNormalW(*_state));
                glEnd();
        	}
        }
        glPopMatrix();
    }

    if( (DRAW_BODY_FORCES & _drawMask) && info._state != NULL ){
    	const RWPEBodyConstraintGraph::DynamicBodyList bodies = _manager->getDynamicBodies();
        const RWPEBodyConstraintGraph::ConstraintList constraints = _manager->getTemporaryConstraints(_state);
        BOOST_FOREACH(const RWPEBodyDynamic* const body, bodies) {
        	const Wrench6D<> wrench = body->getExternalWrench(_gravity,constraints,*_state,*info._state);
        	const Vector3D<> pos = body->getWorldTcom(*_state).P();
            glLineWidth(2.5);
            glBegin(GL_LINES);
            glColor3f(1.0, 0.0, 0.0);
            DrawableUtil::drawGLVertex(pos);
            glColor3f(0.0, 0.0, 0.0);
            DrawableUtil::drawGLVertex(pos + wrench.force()/20);
            glColor3f(1.0, 0.0, 0.0);
            DrawableUtil::drawGLVertex(pos);
            DrawableUtil::drawGLVertex(pos + wrench.torque()/20);
            glEnd();

        }
    }
}

void RWPEDebugRender::update(const RWPEBodyConstraintGraph* manager, const RWPEIslandState* state, const Vector3D<>& gravity) {
	_manager = manager;
	_state = state;
	_gravity = gravity;
}