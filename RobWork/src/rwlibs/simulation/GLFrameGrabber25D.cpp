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


#include "GLFrameGrabber25D.hpp"

#include <rwlibs/os/rwgl.hpp>
#include <boost/foreach.hpp>
#include <rw/math/Math.hpp>

#include <cmath>

using namespace rwlibs::simulation;
using namespace rwlibs::drawable;
using namespace rw::math;

GLFrameGrabber25D::GLFrameGrabber25D(
    int width, int height, double fov,
    rwlibs::drawable::WorkCellGLDrawer *drawer)
    :
    FrameGrabber25D(width,height),
    _fieldOfView(fov),_drawer(drawer),
    _perspTrans(rw::math::Transform3D<double>::identity()),
    _maxDepth(15),_minDepth(0.5)
{}

void GLFrameGrabber25D::setMaxDepth(double depth){
	if(depth<_minDepth)
		RW_THROW("MaxDepth not allowed to be smaller than MinDepth: "<< depth <<">" <<_minDepth );
	_maxDepth = depth;
}
void GLFrameGrabber25D::setMinDepth(double depth){
	if(depth<_maxDepth)
		RW_THROW("MinDepth not allowed to be larger than MaxDepth: "<< depth <<"<" <<_maxDepth );
	_minDepth = depth;
}



void GLFrameGrabber25D::grab(rw::kinematics::Frame *frame,
                          const rw::kinematics::State& state){
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // change viewport to the width and height of image
    GLint oldDim[4]; // viewport dimensions [ x,y,width,height ]
    glGetIntegerv(GL_VIEWPORT,oldDim); // get viewport dimensions
    glViewport(0,0,_img->getWidth(),_img->getHeight()); // set camera view port
    // set camera perspective in relation to a camera model


    if( _depthData.size()!=_img->getWidth()*_img->getHeight() )
    	_depthData.resize(_img->getWidth()*_img->getHeight());

    glMatrixMode(GL_PROJECTION);
    {
        glPushMatrix();
        glLoadIdentity();
        GLdouble aspect = (GLdouble)_img->getWidth() / (GLdouble)_img->getHeight();
        gluPerspective((GLdouble)_fieldOfView, aspect, (GLdouble)_minDepth, (GLdouble)_maxDepth);
    }

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // we rotate because glReadPixels put the char array in different order
    glRotated(180,0,0,1);
    // render scene
    _drawer->drawCameraView(state, frame);
    // copy rendered scene to image


    glReadPixels(
        0, 0,
        _img->getWidth(), _img->getHeight(),
        GL_DEPTH_COMPONENT, GL_FLOAT, &_depthData[0]);

    GLdouble modelview[16];
    GLdouble projection[16];
    GLint viewport[4];

	glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
	glGetDoublev( GL_PROJECTION_MATRIX, projection );
	glGetIntegerv( GL_VIEWPORT, viewport );
	std::vector<Vector3D<float> >& points = _img->getImageData();
	// now unproject all pixel values
	for(size_t y=0;y<_img->getHeight();y++){
		for(size_t x=0;x<_img->getWidth();x++){
			Vector3D<float> &p = points[x+y*_img->getWidth()];
			double winX=x,winY=y,winZ=_depthData[x+y*_img->getWidth()];
			double posX, posY, posZ;
			gluUnProject( winX, winY, winZ,
					modelview, projection, viewport,
					&posX, &posY, &posZ);
			p(0) = posX;
			p(1) = posY;
			p(2) = posZ;
			//std::cout << p << " <---- (" << winX << "," << winY << "," << winZ << "\n";
		}
	}

    // change viewport settings back
    glViewport(oldDim[0],oldDim[1],oldDim[2],oldDim[3]); // set camera view port

    glMatrixMode(GL_PROJECTION);
    {
        glPopMatrix();
    }
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    /*for(size_t )

    // now unproject all pixel values
    BOOST_FOREACH(float &val, imgData){

    	double z = (far+near)/(far-near) + (1/val)*( (-2*far*near)/(far-near) );

    	val = Math::clamp(val, _minDepth, _maxDepth);
    }*/


}

/*    // Create handle to FrameBuffer
      GLuint fbo;
      glGenFramebuffersEXT(1, &fbo);
      // Bind handle with specific framebuffer
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
      // Create handle to a renderbuffer
      GLuint renderbuffer;
      glGenRenderbuffersEXT(1, &renderbuffer);
      // Bind handle to specific renderbuffer
      glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depthbuffer);
      // Allocate storage to the render buffer
      glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT,
      _img->getWidth(), _img->getHeight());
      // Attach the renderbuffer to the framebuffer
      glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
      GL_RENDERBUFFER_EXT, renderbuffer);
      //

      // Check the state of the framebuffer
      GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);

      // Check how many auxiliary buffers are present (can be used as colour buffers)
      GLuint nr_aux;
      glGetIntegerv(GL_AUX_BUFFERS,&nr_aux);
*/

/*    GLboolean db;
    glGetBooleanv(GL_DOUBLEBUFFER,&db);
    if( db ) std::cout << "DOUBLE BUFFER IS USED..." << std::endl;
    else std::cout << "DOUBLE BUFFER IS NOT USED..." << std::endl;
    GLint buf;
    glGetIntegerv(GL_DRAW_BUFFER, &buf);
    std::cout << "CURRENT BUFFER IS USED: " << buf << std::endl;
    */
