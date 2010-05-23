/*
 * RenderScan.cpp
 *
 *  Created on: 23/05/2010
 *      Author: jimali
 */
#include "RenderScan.hpp"

using namespace rwlibs::drawable;

RenderScan::RenderScan():
		_minDepth(0),
		_maxDepth(10)
{}

RenderScan::~RenderScan(){}

void RenderScan::setScan(const rw::sensor::Image25D& img){
	_img = img;
}

void RenderScan::setScan(const rw::sensor::Scan2D& img){
	_img.resize(img.getWidth(),1);
	_img.getImageData() = img.getImageData();
}

void RenderScan::setScan(float dist){
	_img.resize(1,1);
	_img.getImageData()[0] = dist;
}

void RenderScan::draw(DrawType type, double alpha) const
{
	// ignores drawstate
	glPushMatrix();
	float dist = _maxDepth-_minDepth;
    for(int y=0; y<_img.getHeight(); y++){
    	glBegin(GL_LINE_STRIP);
        for(int x=0; x<_img.getWidth(); x++){
        	Vector3D<float> &v = _img.getImageData()[x+y*_img.getWidth()];
        	float col = Math::clamp( (v[2]-_minDepth)/dist, 0, 1);
            glColor3f(col, 0.0, 1-col);
            glVertex3d(v(0), v(1), v(2));    // Bottom Left
        }
        glEnd();
    }
    glPopMatrix();
}
