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


#include "GLFrameGrabber.hpp"

#include <rwlibs/os/rwgl.hpp>

#include <cmath>

using namespace rwlibs::simulation;
using namespace rwlibs::drawable;


#define USE_FRAMEBUFFERS

#ifndef USE_FRAMEBUFFERS

GLFrameGrabber::GLFrameGrabber(
    int width, int height, double fov,
    rwlibs::drawable::WorkCellGLDrawer *drawer)
    :
    FrameGrabber(width,height,rw::sensor::Image::RGB),
    _fieldOfView(fov),_drawer(drawer),
    _perspTrans(rw::math::Transform3D<double>::identity())
{}

GLFrameGrabber::~GLFrameGrabber(){}

void GLFrameGrabber::grab(rw::kinematics::Frame *frame,
                          const rw::kinematics::State& state){
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // change viewport to the width and height of image
    GLint oldDim[4]; // viewport dimensions [ x,y,width,height ]
    glGetIntegerv(GL_VIEWPORT,oldDim); // get viewport dimensions
    glViewport(0,0,_img->getWidth(),_img->getHeight()); // set camera view port
    // set camera perspective in relation to a camera model


    glMatrixMode(GL_PROJECTION);
    {
        glPushMatrix();
        glLoadIdentity();
        GLdouble aspect = (GLdouble)_img->getWidth() / (GLdouble)_img->getHeight();
        gluPerspective((GLdouble)_fieldOfView, aspect, (GLdouble)0.1, (GLdouble)100);
    }

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // we rotate because glReadPixels put the char array in different order
    glRotated(180,0,0,1);
    // render scene
    _drawer->drawCameraView(state, frame);
    // copy rendered scene to image
    char *imgData = _img->getImageData();



    glReadPixels(
        0, 0,
        _img->getWidth(), _img->getHeight(),
        GL_RGB, GL_UNSIGNED_BYTE, imgData);

    // change viewport settings back
    glViewport(oldDim[0],oldDim[1],oldDim[2],oldDim[3]); // set camera view port

    glMatrixMode(GL_PROJECTION);
    {
        glPopMatrix();
    }
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // the image is mirrored in the x-axis
    for(size_t y=0;y<_img->getHeight();y++){
        for(size_t x=0;x<_img->getWidth()/2;x++){
            for(size_t c=0;c<3;c++){
                int idx = (y*_img->getWidth()+x)*3;
                int idxback = (y*_img->getWidth()+_img->getWidth()-1-x)*3;
                unsigned char tmp = imgData[idx+c];
                imgData[idx+c] = imgData[idxback+c];
                imgData[idxback+c] = tmp;
            }
        }
    }

}
#else


#if defined(RW_WIN32)
// Framebuffer object
PFNGLGENFRAMEBUFFERSEXTPROC 					pglGenFramebuffersEXT = 0;                      // FBO name generation procedure
PFNGLDELETEFRAMEBUFFERSEXTPROC                  pglDeleteFramebuffersEXT = 0;                   // FBO deletion procedure
PFNGLBINDFRAMEBUFFEREXTPROC                     pglBindFramebufferEXT = 0;                      // FBO bind procedure
PFNGLCHECKFRAMEBUFFERSTATUSEXTPROC              pglCheckFramebufferStatusEXT = 0;               // FBO completeness test procedure
PFNGLGETFRAMEBUFFERATTACHMENTPARAMETERIVEXTPROC pglGetFramebufferAttachmentParameterivEXT = 0;  // return various FBO parameters
PFNGLGENERATEMIPMAPEXTPROC                      pglGenerateMipmapEXT = 0;                       // FBO automatic mipmap generation procedure
PFNGLFRAMEBUFFERTEXTURE2DEXTPROC                pglFramebufferTexture2DEXT = 0;                 // FBO texdture attachement procedure
PFNGLFRAMEBUFFERRENDERBUFFEREXTPROC             pglFramebufferRenderbufferEXT = 0;              // FBO renderbuffer attachement procedure
// Renderbuffer object
PFNGLGENRENDERBUFFERSEXTPROC                    pglGenRenderbuffersEXT = 0;                     // renderbuffer generation procedure
PFNGLDELETERENDERBUFFERSEXTPROC                 pglDeleteRenderbuffersEXT = 0;                  // renderbuffer deletion procedure
PFNGLBINDRENDERBUFFEREXTPROC                    pglBindRenderbufferEXT = 0;                     // renderbuffer bind procedure
PFNGLRENDERBUFFERSTORAGEEXTPROC                 pglRenderbufferStorageEXT = 0;                  // renderbuffer memory allocation procedure
PFNGLGETRENDERBUFFERPARAMETERIVEXTPROC          pglGetRenderbufferParameterivEXT = 0;           // return various renderbuffer parameters
PFNGLISRENDERBUFFEREXTPROC                      pglIsRenderbufferEXT = 0;                       // determine renderbuffer object type

#define glGenFramebuffersEXT                        pglGenFramebuffersEXT
#define glDeleteFramebuffersEXT                     pglDeleteFramebuffersEXT
#define glBindFramebufferEXT                        pglBindFramebufferEXT
#define glCheckFramebufferStatusEXT                 pglCheckFramebufferStatusEXT
#define glGetFramebufferAttachmentParameterivEXT    pglGetFramebufferAttachmentParameterivEXT
#define glGenerateMipmapEXT                         pglGenerateMipmapEXT
#define glFramebufferTexture2DEXT                   pglFramebufferTexture2DEXT
#define glFramebufferRenderbufferEXT                pglFramebufferRenderbufferEXT

#define glGenRenderbuffersEXT                       pglGenRenderbuffersEXT
#define glDeleteRenderbuffersEXT                    pglDeleteRenderbuffersEXT
#define glBindRenderbufferEXT                       pglBindRenderbufferEXT
#define glRenderbufferStorageEXT                    pglRenderbufferStorageEXT
#define glGetRenderbufferParameterivEXT             pglGetRenderbufferParameterivEXT
#define glIsRenderbufferEXT                         pglIsRenderbufferEXT
#endif
bool fboSupported = false;
bool fboUsed = false;

void initExtensions(){
	// check if FBO is supported by your video card
	//if(glInfo.isExtensionSupported("GL_EXT_framebuffer_object"))
	{
#if defined(RW_WIN32)

		// get pointers to GL functions
		glGenFramebuffersEXT                     = (PFNGLGENFRAMEBUFFERSEXTPROC)wglGetProcAddress("glGenFramebuffersEXT");
		glDeleteFramebuffersEXT                  = (PFNGLDELETEFRAMEBUFFERSEXTPROC)wglGetProcAddress("glDeleteFramebuffersEXT");
		glBindFramebufferEXT                     = (PFNGLBINDFRAMEBUFFEREXTPROC)wglGetProcAddress("glBindFramebufferEXT");
		glCheckFramebufferStatusEXT              = (PFNGLCHECKFRAMEBUFFERSTATUSEXTPROC)wglGetProcAddress("glCheckFramebufferStatusEXT");
		glGetFramebufferAttachmentParameterivEXT = (PFNGLGETFRAMEBUFFERATTACHMENTPARAMETERIVEXTPROC)wglGetProcAddress("glGetFramebufferAttachmentParameterivEXT");
		glGenerateMipmapEXT                      = (PFNGLGENERATEMIPMAPEXTPROC)wglGetProcAddress("glGenerateMipmapEXT");
		glFramebufferTexture2DEXT                = (PFNGLFRAMEBUFFERTEXTURE2DEXTPROC)wglGetProcAddress("glFramebufferTexture2DEXT");
		glFramebufferRenderbufferEXT             = (PFNGLFRAMEBUFFERRENDERBUFFEREXTPROC)wglGetProcAddress("glFramebufferRenderbufferEXT");
		glGenRenderbuffersEXT                    = (PFNGLGENRENDERBUFFERSEXTPROC)wglGetProcAddress("glGenRenderbuffersEXT");
		glDeleteRenderbuffersEXT                 = (PFNGLDELETERENDERBUFFERSEXTPROC)wglGetProcAddress("glDeleteRenderbuffersEXT");
		glBindRenderbufferEXT                    = (PFNGLBINDRENDERBUFFEREXTPROC)wglGetProcAddress("glBindRenderbufferEXT");
		glRenderbufferStorageEXT                 = (PFNGLRENDERBUFFERSTORAGEEXTPROC)wglGetProcAddress("glRenderbufferStorageEXT");
		glGetRenderbufferParameterivEXT          = (PFNGLGETRENDERBUFFERPARAMETERIVEXTPROC)wglGetProcAddress("glGetRenderbufferParameterivEXT");
		glIsRenderbufferEXT                      = (PFNGLISRENDERBUFFEREXTPROC)wglGetProcAddress("glIsRenderbufferEXT");
#endif
		// check once again FBO extension
		if(glGenFramebuffersEXT && glDeleteFramebuffersEXT && glBindFramebufferEXT && glCheckFramebufferStatusEXT &&
		   glGetFramebufferAttachmentParameterivEXT && glGenerateMipmapEXT && glFramebufferTexture2DEXT && glFramebufferRenderbufferEXT &&
		   glGenRenderbuffersEXT && glDeleteRenderbuffersEXT && glBindRenderbufferEXT && glRenderbufferStorageEXT &&
		   glGetRenderbufferParameterivEXT && glIsRenderbufferEXT)
		{
			fboSupported = fboUsed = true;
			std::cout << "Video card supports GL_EXT_framebuffer_object." << std::endl;
	}
	else
	{
		fboSupported = fboUsed = false;
		std::cout << "Video card does NOT support GL_EXT_framebuffer_object." << std::endl;
	}
	}
}

class GLFrameBuffer {

};

GLFrameGrabber::GLFrameGrabber(
    int width, int height, double fov,
    rwlibs::drawable::WorkCellGLDrawer *drawer)
    :
    FrameGrabber(width,height,rw::sensor::Image::RGB),
    _fieldOfView(fov),_drawer(drawer),
    _perspTrans(rw::math::Transform3D<double>::identity())
{
	GLint var;

	initExtensions();

    // create a texture object
	/*
    glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE); // automatic mipmap generation included in OpenGL v1.4
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
	*/
	// allocate a framebuffer
	_fbId = 0;
	_renderId = 0;
	glGenFramebuffersEXT(1, &_fbId);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, _fbId);

	glGenRenderbuffersEXT(1, &_renderId);
	// select render
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, _renderId);
	// create render storage
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGB8, width, height);
	//Attach color buffer to FBO
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, _renderId);

	// now if we need depth of image we also attach depth render buffer
	glGenRenderbuffersEXT(1, &_renderDepthId);
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, _renderDepthId);
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, width, height);
	 //Attach depth buffer to FBO
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, _renderDepthId);

	//Does the GPU support current FBO configuration?
	GLenum status;
	status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
	switch(status){
	case GL_FRAMEBUFFER_COMPLETE_EXT:
		std::cout<<"good" << std::endl;break;
	default:
		std::cout << "NOT GOOD AT ALLL, status: " << status << std::endl;;
	}

	std::cout << _fbId << " " << _renderId << std::endl;

	glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE_EXT, &var);
	std::cout << "GL_MAX_RENDERBUFFER_SIZE_EXT: " << var << std::endl;

	glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_WIDTH_EXT, &var);
	std::cout << "GL_RENDERBUFFER_WIDTH_EXT: " << var << std::endl;
	glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_HEIGHT_EXT, &var);
	std::cout << "GL_RENDERBUFFER_HEIGHT_EXT: " << var << std::endl;
	glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_INTERNAL_FORMAT_EXT, &var);
	std::cout << "GL_RENDERBUFFER_INTERNAL_FORMAT_EXT: " << var << std::endl;
	glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_RED_SIZE_EXT, &var);
	std::cout << "GL_RENDERBUFFER_RED_SIZE_EXT: " << var << std::endl;
	glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_GREEN_SIZE_EXT, &var);
	std::cout << "GL_RENDERBUFFER_GREEN_SIZE_EXT: " << var << std::endl;
	glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_BLUE_SIZE_EXT, &var);
	std::cout << "GL_RENDERBUFFER_BLUE_SIZE_EXT: " << var << std::endl;
	glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_ALPHA_SIZE_EXT, &var);
	std::cout << "GL_RENDERBUFFER_ALPHA_SIZE_EXT: " << var << std::endl;
	glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_DEPTH_SIZE_EXT, &var);
	std::cout << "GL_RENDERBUFFER_DEPTH_SIZE_EXT: " << var << std::endl;
	glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_STENCIL_SIZE_EXT, &var);
	std::cout << "GL_RENDERBUFFER_STENCIL_SIZE_EXT: " << var << std::endl;
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);

    // attach a texture to FBO color attachement point
    //glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, textureId, 0);

    // attach a renderbuffer to depth attachment point
    //glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, _fbId);

	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}

GLFrameGrabber::~GLFrameGrabber(){
	glDeleteFramebuffersEXT(1, &_fbId);
	glDeleteRenderbuffersEXT(1, &_renderId);
	glDeleteRenderbuffersEXT(1, &_renderDepthId);
}

void GLFrameGrabber::grab(rw::kinematics::Frame *frame,
                          const rw::kinematics::State& state){
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, _fbId);

	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // change viewport to the width and height of image
    GLint oldDim[4]; // viewport dimensions [ x,y,width,height ]
    glGetIntegerv(GL_VIEWPORT,oldDim); // get viewport dimensions
    glViewport(0,0,_img->getWidth(),_img->getHeight()); // set camera view port
    // set camera perspective in relation to a camera model


	//glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, _renderId);

    glMatrixMode(GL_PROJECTION);
    {
        glPushMatrix();
        glLoadIdentity();
        GLdouble aspect = (GLdouble)_img->getWidth() / (GLdouble)_img->getHeight();
        gluPerspective((GLdouble)_fieldOfView, aspect, (GLdouble)0.1, (GLdouble)100);
    }

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // we rotate because glReadPixels put the char array in different order
    glRotated(180,0,0,1);
    // render scene
    _drawer->drawCameraView(state, frame);



    // copy rendered scene to image
    char *imgData = _img->getImageData();
    //glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);

    //glBindTexture(GL_TEXTURE_2D, textureId);

    glReadPixels(
        0, 0,
        _img->getWidth(), _img->getHeight(),
        GL_RGB, GL_UNSIGNED_BYTE, imgData);
    //glBindTexture(GL_TEXTURE_2D, 0);

    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);


    // change viewport settings back
    glViewport(oldDim[0],oldDim[1],oldDim[2],oldDim[3]); // set camera view port

    glMatrixMode(GL_PROJECTION);
    {
        glPopMatrix();
    }
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // the image is mirrored in the x-axis
    for(size_t y=0;y<_img->getHeight();y++){
        for(size_t x=0;x<_img->getWidth()/2;x++){
            for(size_t c=0;c<3;c++){
                int idx = (y*_img->getWidth()+x)*3;
                int idxback = (y*_img->getWidth()+_img->getWidth()-1-x)*3;
                unsigned char tmp = imgData[idx+c];
                imgData[idx+c] = imgData[idxback+c];
                imgData[idxback+c] = tmp;
            }
        }
    }

    //glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}


#endif
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
