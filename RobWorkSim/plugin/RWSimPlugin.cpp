#include "RWSimPlugin.hpp"

#ifdef __WIN32
#include <windows.h>
#endif

#include <iostream>

#include <boost/foreach.hpp>

#include <RobWorkStudio.hpp>

#include <rw/math/RPY.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <rw/common/TimerUtil.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/proximity/Proximity.hpp>
#include <rw/loaders/path/PathLoader.hpp>

#include "JointControlDialog.hpp"

#include <dynamics/RigidBody.hpp>
#include <loaders/DynamicWorkCellLoader.hpp>
#include <simulator/SimulatorDebugRender.hpp>
#include <simulator/PhysicsEngineFactory.hpp>

#include <dynamics/KinematicDevice.hpp>
#include <dynamics/RigidDevice.hpp>

#include <rw/sensor/TactileArray.hpp>
#include <rwlibs/drawable/TactileArrayRender.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwlibs/simulation/SimulatedSensor.hpp>
#include <rwlibs/control/JointController.hpp>


#include <control/PDController.hpp>
#include <control/VelRampController.hpp>

#include "SimCfgDialog.hpp"
#include "CreateEngineDialog.hpp"

using namespace rw::loaders;
using namespace rw::trajectory;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::proximity;
using namespace rw::sensor;
using namespace rwlibs::drawable;
using namespace rwlibs::simulation;

using namespace dynamics;
using namespace drawable;
using namespace loaders;

#define RW_DEBUGS( str ) //std::cout << str  << std::endl;

RWSimPlugin::RWSimPlugin():
	RobWorkStudioPlugin("RWSimPlugin", QIcon(":/SimulationIcon.png")),
	_dwc(NULL),
	_sim(NULL),
	_debugRender(NULL),
	_tactileSensorDialog(NULL)
{
    setupUi(this);

    connect(_openDwcBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_openLastDwcBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_editDwcBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_createSimulatorBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_destroySimulatorBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_simConfigBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_stepBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_startBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(_resetBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_stopBtn     ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_saveStatePathBtn,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(_tactileSensorBtn,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(_openDeviceCtrlBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(_timeStepSpin  ,SIGNAL(valueChanged(double)), this, SLOT(changedEvent()) );
    connect(_timeScaleSpin  ,SIGNAL(valueChanged(double)), this, SLOT(changedEvent()) );
    connect(_debugLevelSpin ,SIGNAL(valueChanged(int)), this, SLOT(changedEvent()) );
    connect(_updateIntervalSpin ,SIGNAL(valueChanged(double)), this, SLOT(changedEvent()) );

    connect(_debugLevelSpin , SIGNAL(valueChanged(int)), this, SLOT(changedEvent()));


    // seed generat
    Math::seed( clock() );

    _timer = new QTimer( NULL );
    _timer->setInterval( (int)(_updateIntervalSpin->value()*1000) );
    connect( _timer, SIGNAL(timeout()), this, SLOT(changedEvent()) );
}

void RWSimPlugin::btnPressed(){
    QObject *obj = sender();
    std::cout << "Bumbum" << std::endl;
    if( obj == _openDwcBtn ){
    	openDwc("");
    	if(_dwc==NULL) return;
    	_closeDwcBtn->setDisabled(false);
    	_openDwcBtn->setDisabled(true);
    	_openLastDwcBtn->setDisabled(true);
    } else if( obj == _timerShot ) {
        std::cout << "timer shot" << std::endl;
        std::string str = getRobWorkStudio()->getPropertyMap().get<std::string>("Arg1");
        openDwc(str);
        if( _dwc==NULL ) return;
        _closeDwcBtn->setDisabled(false);
        _openDwcBtn->setDisabled(true);
        _openLastDwcBtn->setDisabled(true);

    } else if( obj == _openLastDwcBtn ) {
    	//openDwc("C:/workspace/RobWorkApp/rwsim/scenes/PumpeHusOnFloor/DWPumpeHuse1x1.xml");
        openDwc("C:/workspace/RobWorkApp/rwsim/scenes/KTHGraspTest/DWCSetup.xml");

        if( _dwc==NULL ) return;
    	_closeDwcBtn->setDisabled(false);
    	_openDwcBtn->setDisabled(true);
    	_openLastDwcBtn->setDisabled(true);
    } else if( obj == _closeDwcBtn ) {
    	_closeDwcBtn->setDisabled(true);
    	_openDwcBtn->setDisabled(false);
    	_openLastDwcBtn->setDisabled(false);
    	getRobWorkStudio()->setWorkcell( NULL );
    	_dwc = NULL;
    } else if( obj == _editDwcBtn ) {

    } else if( obj == _createSimulatorBtn ) {
        // Open a create simulator dialog
    	CreateEngineDialog eDialog(_dwc,this);
    	eDialog.exec();

    	Simulator *sim = eDialog.getSimulator();
    	if(sim==NULL)
    		return;
    	_createSimulatorBtn->setDisabled(true);
    	_destroySimulatorBtn->setDisabled(false);
    	_simConfigBtn->setDisabled(false);
    	State state = getRobWorkStudio()->getState();
    	sim->initPhysics(state);
    	_sim = ownedPtr(new ThreadSimulator(sim, getRobWorkStudio()->getState()));
    	_timer->start();
    } else if( obj == _destroySimulatorBtn ) {
    	_sim->stop();
    	_sim->getSimulator()->exitPhysics();
    	_sim = NULL;
    	_destroySimulatorBtn->setDisabled(true);
    	_createSimulatorBtn->setDisabled(false);
    	_simConfigBtn->setDisabled(true);
    	_timer->stop();
    } else if( obj == _simConfigBtn ) {
    	if(!_sim){
    		RW_WARN("No simulator created yet!");
    		return;
    	}
    	SimCfgDialog eDialog(_sim->getSimulator(),this);
    	eDialog.exec();

    } else if( obj == _stepBtn ) {
    	if( _sim->isRunning() )
    		_sim->stop();
    	_sim->step();
    	getRobWorkStudio()->setState(_sim->getState());
    } else if( obj == _startBtn ) {
    	_startBtn->setDisabled(true);
    	_stopBtn->setDisabled(false);
    	_sim->start();
    } else if( obj == _stopBtn ) {
    	_sim->stop();
    	_startBtn->setDisabled(false);
    	_stopBtn->setDisabled(true);
    } else if( obj == _resetBtn ) {
    	_sim->setState( getRobWorkStudio()->getState() );
    } else if( obj == _saveStatePathBtn )  {

    } else if( obj == _openDeviceCtrlBtn ){
        std::string devname = _deviceControlBox->currentText().toStdString();
        DynamicDevice *ddev = _dwc->findDevice(devname);
        RW_ASSERT(ddev);
        State state = getRobWorkStudio()->getState();
        // create a joint controller for the device and add it too the simulator
        JointControllerPtr jctrl;
        if( dynamic_cast<KinematicDevice*>(ddev) ){
            KinematicDevice *kdev = dynamic_cast<KinematicDevice*>(ddev);
            VelRampControllerPtr vctrl = ownedPtr(new VelRampController(kdev, state));
            _sim->getSimulator()->addController(vctrl);
            jctrl = vctrl;
        } else if ( dynamic_cast<RigidDevice*>(ddev) ){
            RigidDevice *rdev = dynamic_cast<RigidDevice*>(ddev);
            PDControllerPtr pdctrl = ownedPtr(new PDController(rdev,state));
            _sim->getSimulator()->addController(pdctrl);
            jctrl = pdctrl;
        }


        JointControlDialog *dialog = new JointControlDialog(jctrl,this);
        dialog->show();
        dialog->raise();
        dialog->activateWindow();
    } else if( obj==_tactileSensorBtn ){
        if( _tactileSensorDialog==NULL )
            _tactileSensorDialog = new TactileSensorDialog(_dwc.get(), this);
        connect(this,SIGNAL(updateDialog()),_tactileSensorDialog,SLOT(updateState()) );
        _tactileSensorDialog->show();
        _tactileSensorDialog->raise();
        _tactileSensorDialog->activateWindow();
    }
}

void RWSimPlugin::changedEvent(){
    QObject *obj = sender();
    if( obj == _timer ){
        // update stuff
        updateStatus();
    } else if( obj == _timeStepSpin ){
    	_sim->setTimeStep( _timeStepSpin->value() );
    } else if( obj == _updateIntervalSpin ) {
    	_timer->setInterval( (int)(_updateIntervalSpin->value()*1000) );
    } else if( obj == _timeScaleSpin ){
    	_sim->setPeriodMs( (int)(_timeScaleSpin->value()*_timeStepSpin->value()*1000) );
    } else if( obj == _debugLevelSpin ){
    	std::cout << "Debug level spin!!" << std::endl;
    	if(_debugRender==NULL){
    		if( _debugLevelSpin->value()==0 )
    			return;
    		SimulatorDebugRender *render = _sim->getSimulator()->createDebugRender();
    		if( render == NULL ){
    			Log::errorLog() << "The current simulator does not support debug rendering!" << std::endl;
    			return;
    		}
            render->setDrawMask( _debugLevelSpin->value() );
    		_debugRender = new Drawable( ownedPtr(render) );
            getRobWorkStudio()->getWorkCellGLDrawer()->
				addDrawableToFrame(_dwc->getWorkcell()->getWorldFrame(), _debugRender);
    	}

	}
}

void RWSimPlugin::updateStatus(){
	if(_sim==NULL)
		return;

	if( _forceSceneUpdate->isChecked() && _sim->isRunning() ){
		getRobWorkStudio()->setState(_sim->getState());
	}

	double time = _sim->getTime();
	std::stringstream str;
	str << time << " s";
	_timeLbl->setText(str.str().c_str());
}

void RWSimPlugin::open(rw::models::WorkCell* workcell){
	if( workcell==NULL || _dwc==NULL )
		return;

	// add sensor drawables to the workcell drawer
    BOOST_FOREACH(SimulatedSensorPtr sensor,  _dwc->getSensors()){
        if( dynamic_cast<TactileArray*>(sensor.get()) ){
            //std::cout << "ADDING TACTILE SENSOR DRAWER..." << std::endl;
            TactileArray *tsensor = dynamic_cast<TactileArray*>(sensor.get());
            TactileArrayRender *render = new TactileArrayRender(tsensor);
            Drawable *drawable = new Drawable(ownedPtr<Render>(render));
            //getRobWorkStudio()->getWorkCellGLDrawer()->addDrawableToFrame(workcell->getWorldFrame(), drawable);
            //std::cout << "TO: " << sensor->getFrame()->getName() << std::endl;
            getRobWorkStudio()->getWorkCellGLDrawer()->addDrawableToFrame(tsensor->getFrame(), drawable);
        }
    }

    _deviceControlBox->clear();
    BOOST_FOREACH(DynamicDevice* device, _dwc->getDynamicDevices()){
        rw::models::Device *dev = &device->getModel();
        RW_ASSERT(dev);
        std::cout << "Dev name: " << std::endl;
        std::cout << dev->getName() << std::endl;
        _deviceControlBox->addItem(dev->getName().c_str());
    }
}

void RWSimPlugin::openDwc(const std::string& file){
	std::string dwcFile;
	if( file.empty() ){
	    QString selectedFilter;
	    const QString dir(_context._previousOpenDirectory.c_str());

	    QString filename = QFileDialog::getOpenFileName(
	        this,
	        "Open Drawable", // Title
	        dir, // Directory
	        "All supported ( *.xml )"
	        " \nRW XML files ( *.xml )"
	        " \n All ( *.* )",
	        &selectedFilter);

	    dwcFile = filename.toStdString();
	} else {
		dwcFile = file;
	}
	if (dwcFile.empty())
		return;

    _context._previousOpenDirectory =
    	rw::common::StringUtil::getDirectoryName(dwcFile);

    Ptr<DynamicWorkcell> dwc(NULL);
    try {
        dwc = DynamicWorkCellLoader::load(dwcFile);
    } catch (const Exception& exp) {
        QMessageBox::information(
            NULL,
            "Exception",
            exp.getMessage().getText().c_str(),
            QMessageBox::Ok);
        return;
    }

    if( dwc==NULL )
    	RW_THROW("Dynamic workcell is null");

    _dwc = dwc;

    std::vector<DynamicDevice*> devs = _dwc->getDynamicDevices();
    BOOST_FOREACH(DynamicDevice* dev, devs){
    	std::cout << dev->getModel().getName() << std::endl;
    }


    getRobWorkStudio()->setWorkcell( dwc->getWorkcell() );
    // adding the DynamicWorkcell to the propertymap such that others can use it
    getRobWorkStudio()->getPropertyMap().add<Ptr<DynamicWorkcell> >(
            "DynamicWorkcell",
            "A workcell with dynamic description",
            _dwc );

    // signal to other plugins that a DynamicWorkCell has been loadet
    getRobWorkStudio()->genericEvent().fire("DynamicWorkcellLoadet");
}

void RWSimPlugin::close(){

}

void RWSimPlugin::initialize(){
    getRobWorkStudio()->stateChangedEvent().add(
    		boost::bind(&RWSimPlugin::stateChangedListener, this, _1), this);

    _timerShot = new QTimer( NULL );

    if( getRobWorkStudio()->getPropertyMap().has("Arg1") ){
        _timerShot->setSingleShot(true);
        _timerShot->setInterval( 6000 );
        _timerShot->start();
        connect( _timerShot, SIGNAL(timeout()), this, SLOT(btnPressed()) );

        //getRobWorkStudio()->getView()->setZoomScale(5.0);
        getRobWorkStudio()->getView()->setCheckForCollision(false);
    } else {
        std::cout << "NOOOOO ARG" << std::endl;
    }
}

void RWSimPlugin::stateChangedListener(const State& state){
    updateDialog();
}

Q_EXPORT_PLUGIN(RWSimPlugin);
