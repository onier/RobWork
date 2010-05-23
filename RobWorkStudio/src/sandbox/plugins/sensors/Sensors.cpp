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

#include "Sensors.hpp"

#include <rws/RobWorkStudio.hpp>

#include <sstream>

#include <boost/foreach.hpp>

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/use_robwork_namespace.hpp>

#include <rwlibs/simulation/camera/SimulatedCamera.hpp>
#include <rwlibs/simulation/SimulatedScanner25D.hpp>
//#include <rwlibs/simulation/SimulatedScanner2D.hpp>
//#include <rwlibs/simulation/SimulatedScanner1D.hpp>

#include <rw/sensor.hpp>

using namespace robwork;

using namespace rw::sensor;
using namespace rwlibs::simulation;
using namespace rwlibs::drawable;
using namespace rws;
#include <sstream>


Sensors::Sensors()
    :
    RobWorkStudioPlugin("Sensors", QIcon(":/lua.png")),
    _camera(NULL),
    _scanner25d(NULL),
    _scanner2d(NULL),
    _scanner1d(NULL)
{
    QWidget *widget = new QWidget(this);
    QVBoxLayout *lay = new QVBoxLayout(widget);
    widget->setLayout(lay);
    this->setWidget(widget);

    {
        QPushButton* button = new QPushButton("Grab Image");
        lay->addWidget(button); // Own button.
        connect(button, SIGNAL(pressed()), this, SLOT(grabImage()));
    }
    {
        QPushButton* button = new QPushButton("Grab Scan25D");
        lay->addWidget(button); // Own button.
        connect(button, SIGNAL(pressed()), this, SLOT(grabScan25D()));
    }

    {
        QPushButton* button = new QPushButton("Grab Scan2D");
        lay->addWidget(button); // Own button.
        connect(button, SIGNAL(pressed()), this, SLOT(grabScan2D()));
    }
    {
        QPushButton* button = new QPushButton("Grab Scan1D");
        lay->addWidget(button); // Own button.
        connect(button, SIGNAL(pressed()), this, SLOT(grabScan1D()));
    }

	_timer = new QTimer(this);
	connect(_timer, SIGNAL(timeout()), this, SLOT(updateSim()));
	_timer->start(100);
}

Sensors::~Sensors()
{

}

void Sensors::initialize()
{
    getRobWorkStudio()->stateChangedEvent().add(
        boost::bind(
            &Sensors::stateChangedListener,
            this,_1), this);
}


void Sensors::updateSim(){
    // first we update all sensors
    BOOST_FOREACH(SimulatedSensor *sensor, _simSensors){
    	sensor->update(0.01, _state);
    }

}

void Sensors::stateChangedListener(const State& state)
{
    _state = state;

	// take image with camera
    //if(_camera)
    //	_camera->
}

void Sensors::grabImage(){
	log().info() << "Grab image";
	if( _camera->isImageReady() ){
		std::cout << "Image is ready!" << std::endl;
		const Image* img = _camera->getImage();
		_imgRender->setImage(*img);
	}
	_camera->acquire();

}

void Sensors::grabScan25D(){
	log().info() << "Scan 25";
	if( _scanner25d->isScanReady() ){
		std::cout << "image ready!" << std::endl;
		const Image25D& img = _scanner25d->getImage();
		_scanRender->setScan(img);
	}

	_scanner25d->acquire();
}

void Sensors::grabScan2D(){
	log().info() << "Scan 25";
}

void Sensors::grabScan1D(){
	log().info() << "Scan 25";
}
#include <rwlibs/drawable/RenderUtil.hpp>
void Sensors::open(WorkCell* workcell)
{
	_simSensors.clear();
	if(_camera!=NULL)
		delete _camera;
	if(_scanner25d!=NULL)
		delete _scanner25d;
	if(_scanner2d!=NULL)
		delete _scanner2d;
	if(_scanner1d!=NULL)
		delete _scanner1d;
	State state = getRobWorkStudio()->getState();

	WorkCellGLDrawer *gldrawer = getRobWorkStudio()->getWorkCellGLDrawer();

	// first we do the camera
	_framegrabber = ownedPtr( new GLFrameGrabber(640,480,50,gldrawer) );
	SimulatedCamera *simcam = new SimulatedCamera("SimCam1",_framegrabber);
	_camera = simcam;
	_simSensors.push_back(simcam);

	// then we do the scanner25d
	_framegrabber25d = ownedPtr( new GLFrameGrabber25D(640,480,50,gldrawer) );
	SimulatedScanner25D *simscan25 = new SimulatedScanner25D("SimScan25D1",_framegrabber25d);
	_scanner25d = simscan25;
	_simSensors.push_back(simscan25);

	Frame *frame = NULL;
	Frame *world = workcell->getWorldFrame();
	frame = workcell->findFrame("Camera");
	if(frame) _camera->attachTo(frame);
	else _camera->attachTo(world);

	frame = workcell->findFrame("Scanner25D");
	if(frame) _scanner25d->attachTo(frame);
	else _scanner25d->attachTo(world);
	// and drawables.

	_cameraViewRender = RenderUtil::makeCameraViewRender(640, 480, 50);
	_scanViewRender = RenderUtil::makeCameraViewRender(640, 480, 50);

	gldrawer->addDrawableToFrame(_camera->getFrame(), new Drawable(_cameraViewRender));
	gldrawer->addDrawableToFrame(_scanner25d->getFrame(), new Drawable(_scanViewRender));

	frame = workcell->findFrame("ScanScreen");
	if(!frame) frame = world;
	_scanRender = ownedPtr( new RenderScan() );
	gldrawer->addDrawableToFrame(frame, new Drawable(_scanRender));

	frame = workcell->findFrame("Screen");
	if(!frame) frame = world;

	_imgRender = ownedPtr(new RenderImage());
	gldrawer->addDrawableToFrame(frame, new Drawable(_imgRender));

	// remember to initialize camera and scanners
	_camera->initialize();
	_camera->start();
	_scanner25d->open();

    stateChangedListener(getRobWorkStudio()->getState());
}

void Sensors::close()
{}

//----------------------------------------------------------------------
#ifndef RW_STATIC_LINK_PLUGINS
Q_EXPORT_PLUGIN2(Sensors, Sensors)
#endif
