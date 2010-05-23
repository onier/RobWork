/*
 * SimulatedScanner25D.cpp
 *
 *  Created on: 23/05/2010
 *      Author: jimali
 */

#include "SimulatedScanner25D.hpp"

using namespace rwlibs::simulation;
using namespace rw::sensor;

SimulatedScanner25D::SimulatedScanner25D(
		const std::string& name,
		FrameGrabber25DPtr framegrabber):
		Scanner25D(name, "Simulated Scanner25D"),
		_framegrabber(framegrabber),
		_frameRate(30)
{}

SimulatedScanner25D::SimulatedScanner25D(const std::string& name,
		const std::string& desc,
		FrameGrabber25DPtr framegrabber):
		Scanner25D(name,desc),
		_framegrabber(framegrabber),
		_frameRate(30)
{}

SimulatedScanner25D::~SimulatedScanner25D(){}


void SimulatedScanner25D::open(){
	_isOpenned = true;
}

bool SimulatedScanner25D::isOpen(){
	return _isOpenned;
}

void SimulatedScanner25D::close(){
	_isOpenned = false;
}

void SimulatedScanner25D::acquire(){
	if(!_isOpenned)
		RW_THROW("Scanner has not been openned yet!");
	_isAcquired = false;
}

bool SimulatedScanner25D::isScanReady(){
	return _isAcquired;
}

std::pair<double,double> SimulatedScanner25D::getRange(){
	return std::make_pair(_framegrabber->getMinDepth(),_framegrabber->getMaxDepth());
}

double SimulatedScanner25D::getFrameRate(){
	return _frameRate;
}

const Image25D& SimulatedScanner25D::getImage(){
	return _framegrabber->getImage();
}

void SimulatedScanner25D::update(double dt, rw::kinematics::State& state){
    if(!_isOpenned || _isAcquired)
        return;
    if( _frameRate<0.00001 )
    	return;

    _dtsum += dt;

    if( _dtsum>1.0/_frameRate ){
    	_dtsum = 0;
    	_framegrabber->grab(getFrame(), state);
    	_isAcquired = true;
    }

}

void SimulatedScanner25D::reset(const rw::kinematics::State& state){

}

rw::sensor::Sensor* SimulatedScanner25D::getSensor(){return this;};
