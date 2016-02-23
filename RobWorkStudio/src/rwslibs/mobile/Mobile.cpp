/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "Mobile.hpp"

#include <iostream>
#include <sstream>
#include <typeinfo>

#include <boost/bind.hpp>

#include <QMessageBox>

#include <rws/RobWorkStudio.hpp>

#include <rw/math/RPY.hpp>
#include <rw/math/Constants.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/models/Joint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/PseudoOmniDevice.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::graphics;

using namespace rws;

namespace {
    /*
     * This function takes string descriptions of the desired angular and translational units.
     * These units are then matched (case insensitive) to the known units and a correctly parsed
     * pair of unit descriptions are returned (default is <Radians, Meters>).
     */
    std::pair<std::string, std::string> formatUnitDescriptions(const std::string& angles,
                                                               const std::string& distances,
                                                               const std::map<std::string, double>& angleUnitConverters,
                                                               const std::map<std::string, double>& distanceUnitConverters) {
        // Default values if matches are not found
        std::pair<std::string, std::string> descs("Radians", "Meters");

        // Angles
        for(std::map<std::string, double>::const_iterator it = angleUnitConverters.begin(); it != angleUnitConverters.end(); ++it)
            if(QString::compare(QString(angles.c_str()), QString(it->first.c_str()), Qt::CaseInsensitive) == 0)
                descs.first = it->first;

        // Distances
        for(std::map<std::string, double>::const_iterator it = distanceUnitConverters.begin(); it != distanceUnitConverters.end(); ++it)
            if(QString::compare(QString(distances.c_str()), QString(it->first.c_str()), Qt::CaseInsensitive) == 0)
                descs.second = it->first;

        return descs;
    }

    /*
     * Inputs are the desired angular/translational unit descriptions, a map of descriptions/converters and a device pointer.
     * First the descriptions are corrected (capital first letter) and then the unit data is created.
     * If a device is input, a vector of angular <descriptions, converters> are returned for each joint.
     * Else the first three DOF designate the translational part and the last three DOF the angular part.
     */
    std::pair<std::vector<std::string>, std::vector<double> > makeSliderUnitData(const std::string& angles,
                                                                                 const std::string& distances,
                                                                                 const std::map<std::string, double>& angleUnitConverters,
                                                                                 const std::map<std::string, double>& distanceUnitConverters,
																				 const rw::models::Device::Ptr selectedDevice = 0) {
        const std::pair<std::string, std::string> descs = formatUnitDescriptions(angles, distances, angleUnitConverters, distanceUnitConverters);
        std::vector<std::string> desc;
        std::vector<double> conv;
        if(selectedDevice) { // Device
            // TODO: this whole thing --------------------------------------------------
            // Cast to joint device to get joint info
			rw::models::JointDevice::Ptr jointDevice = selectedDevice.cast<rw::models::JointDevice>();
            if(jointDevice) {
              // Get joints
              const std::vector<rw::models::Joint*>& joints = jointDevice->getJoints();
              // Iterate through
              for(std::vector<rw::models::Joint*>::const_iterator it = joints.begin(); it != joints.end(); ++it) {
                if(dynamic_cast<const rw::models::RevoluteJoint*>(*it)) { // Revolute joint
                  // Insert angle converter
                  desc.insert(desc.end(), descs.first);
                  const double toUnit = angleUnitConverters.find(descs.first)->second;
                  conv.insert(conv.end(), toUnit);
                } else if(dynamic_cast<const rw::models::PrismaticJoint*>(*it)) { // Prismatic joint
                  // Insert distance converter
                  desc.insert(desc.end(), descs.second);
                  const double toUnit = distanceUnitConverters.find(descs.second)->second;
                  conv.insert(conv.end(), toUnit);
                }/* else if(dynamic_cast<const rw::models::BeamJoint*>(*it)) { // Beam joint
                  // Insert distance converter
                  desc.insert(desc.end(), descs.second);
                  double toUnit = distanceUnitConverters.find(descs.second)->second;
                  conv.insert(conv.end(), toUnit);
                  // Insert angle converter
                  desc.insert(desc.end(), descs.first);
                  toUnit = angleUnitConverters.find(descs.first)->second;
                  conv.insert(conv.end(), toUnit);                  
                  
                }*/ // TODO
              }
            } else {
                // create unit converter that does nothing
            	for (std::size_t i = 0; i < selectedDevice->getDOF(); i++) {
            		conv.push_back(1.0);
            		desc.push_back("None");
            	}
            }
            // --------------------------------------------------
            /*
            desc.insert(desc.end(), selectedDevice->getDOF(), descs.first);
            const double toUnit = angleUnitConverters.find(descs.first)->second;
            conv.insert(conv.end(), selectedDevice->getDOF(), toUnit);
            */
        } else { // Cartesian
            desc.insert(desc.end(), 3, descs.first);
            desc.insert(desc.end(), 3, descs.second);
            const double toUnitDistance = distanceUnitConverters.find(descs.second)->second;
            conv.insert(conv.end(), 3, toUnitDistance);
            const double toUnitAngle = angleUnitConverters.find(descs.first)->second;
            conv.insert(conv.end(), 3, toUnitAngle);
        }
        // if there is no elements in these then it will fail later on
        RW_ASSERT(desc.size()>0);
        RW_ASSERT(conv.size()>0);
        return std::pair<std::vector<std::string>, std::vector<double> >(desc, conv);
    }

    /*
     * Update a unit combo box index by searching (case insensitive) for the current unit description set.
     */
    void updateUnitCB(QComboBox* cmbUnits, const std::string& desc) {
        int index = cmbUnits->findText(QObject::tr(desc.c_str()), static_cast<Qt::MatchFlags>(Qt::MatchFixedString));
        if(index == -1)
            index = 0;
        cmbUnits->setCurrentIndex(index);
    }
}

QIcon Mobile::getIcon() {
  //  Q_INIT_RESOURCE(resources);
    return QIcon(":/mobile.png");
}

Mobile::Mobile():
    RobWorkStudioPlugin("Mobile", getIcon()),
    _workcell(0),
    _selectedDevice(0),
    _updating(false),
    _cartesianBounds(Q::zero(6), Q::zero(6)),
    _rwsSettings(0)
{
    QScrollArea *widg = new QScrollArea(this);
    widg->setWidgetResizable(true);
    QWidget* base = new QWidget(this);
    QGridLayout* pLayout = new QGridLayout(base);
    base->setLayout(pLayout);

    int row = 0;

    _cmbDevices = new QComboBox();

    pLayout->addWidget(_cmbDevices, row++, 0);
    connect(_cmbDevices, SIGNAL(currentIndexChanged(int)), this, SLOT(cmbChanged ( int )));

    _cmbAngleUnit = new QComboBox();
    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(tr("Angle unit:")));
    hbox->addWidget(_cmbAngleUnit);
    pLayout->addLayout(hbox, row++, 0);

    _cmbDistanceUnit = new QComboBox();
    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(tr("Distance unit:")));
    hbox->addWidget(_cmbDistanceUnit);
    pLayout->addLayout(hbox, row++, 0);

    // TODO: Hard-coded map of known units and their converters (base: Radians/Meters).
    _angleUnitConverters.insert(std::pair<std::string, double>("Radians", 1.0));
    _angleUnitConverters.insert(std::pair<std::string, double>("Degrees", rw::math::Rad2Deg));
    _angleUnitConverters.insert(std::pair<std::string, double>("Grads", rw::math::Rad2Deg*4.0/3.6));
    _angleUnitConverters.insert(std::pair<std::string, double>("Turns", 0.5/rw::math::Pi));

    for(std::map<std::string, double>::const_iterator it = _angleUnitConverters.begin(); it != _angleUnitConverters.end(); ++it)
        _cmbAngleUnit->addItem(tr(it->first.c_str()));

    _distanceUnitConverters.insert(std::pair<std::string, double>("Meters", 1.0));
    _distanceUnitConverters.insert(std::pair<std::string, double>("Centimeters", 100.0));
    _distanceUnitConverters.insert(std::pair<std::string, double>("Millimeters", 1000.0));
    _distanceUnitConverters.insert(std::pair<std::string, double>("Inches", rw::math::Meter2Inch));

    for(std::map<std::string, double>::const_iterator it = _distanceUnitConverters.begin(); it != _distanceUnitConverters.end(); ++it)
        _cmbDistanceUnit->addItem(tr(it->first.c_str()));

    connect(_cmbAngleUnit, SIGNAL(currentIndexChanged(int)), this, SLOT(cmbUnitChanged( int )));
    connect(_cmbDistanceUnit, SIGNAL(currentIndexChanged(int)), this, SLOT(cmbUnitChanged( int )));

    _tabWidget = new QTabWidget();
    pLayout->addWidget(_tabWidget, row++, 0); // own _tabWidget

    connect(_tabWidget, SIGNAL(currentChanged(int)), this, SLOT(tabChanged(int)));

    pLayout->setRowStretch(row, 1);


    for (int i = 0; i<3; i++) {
        _cartesianBounds.first(i) = -5;
        _cartesianBounds.second(i) = 5;
        _cartesianBounds.first(i+3) = -2*Pi;
        _cartesianBounds.second(i+3) = 2*Pi;
    }

    widg->setWidget(base);

    //QHBoxLayout *formLayout = new QHBoxLayout;
    //formLayout->addWidget(widg);
    //this->setLayout(formLayout);
    this->setWidget(widg);
}

Mobile::~Mobile()
{
}

void Mobile::initialize()
{
    getRobWorkStudio()->stateChangedEvent().add(
    		boost::bind(&Mobile::stateChangedListener, this, _1), this);

    getRobWorkStudio()->frameSelectedEvent().add(
    		boost::bind(&Mobile::frameSelectedListener, this, _1), this);
}


void Mobile::open(WorkCell* workcell)
{
	typedef std::vector<Device::Ptr>::const_iterator DevI;
    typedef std::vector<Frame*> FrameVector;

	const std::vector<Device::Ptr>& devices = workcell->getDevices();
    close();
    _workcell = workcell;
    if (workcell) {
        //std::cout<<"Get State"<<std::endl;
        _state = getRobWorkStudio()->getState();
        int qs_pos = 0;
        for (DevI it = devices.begin(); it != devices.end(); ++it, ++qs_pos) {
        	if (const PseudoOmniDevice::Ptr mdev = (*it).cast<PseudoOmniDevice>()) {
        		std::string name = mdev->getName();
        		_items.push_back(mdev);
        		_chosenTabs.push_back(0);
        		_frameToIndex[*(mdev->getBase())] = _items.size()-1;
        		QVariant qvar((int)(_items.size()-1));
        		_cmbDevices->addItem(name.c_str(), qvar);
        	}
        }
        _renderICM = new RenderICM();
        getRobWorkStudio()->getWorkCellScene()->addRender("MobileRenderICM",_renderICM,_workcell->getWorldFrame());
    } else {
        close();
    }
    _rwsSettings = getRobWorkStudio()->getPropertyMap().getPtr<rw::common::PropertyMap>("RobWorkStudioSettings");
    if(_rwsSettings) {
        // Find the unit properties if there are any
        std::string unitDescAngle = _rwsSettings->get<std::string>("AngleUnit", "");
        std::string unitDescDistance = _rwsSettings->get<std::string>("DistanceUnit", "");
        // Update combo boxes
        if(unitDescAngle.size()) {
            updateUnitCB(_cmbAngleUnit, unitDescAngle);
            unitDescAngle = _cmbAngleUnit->currentText().toStdString();
        }
        if(unitDescDistance.size()) {
            updateUnitCB(_cmbDistanceUnit, unitDescDistance);
            unitDescDistance = _cmbDistanceUnit->currentText().toStdString();
        }

        updateUnit(unitDescAngle, unitDescDistance);
    }
}

void Mobile::cmbChanged ( int index ) {
    if(index<0)
        return;
    std::string str = _cmbDevices->currentText().toStdString();
    int n = _cmbDevices->itemData(index).toInt();
    disconnect(_tabWidget, 0, 0, 0);
    if (_items[n] != NULL) {
        removeTabs();
        _selectedDevice = _items[n];
        constructTabs(_selectedDevice);
    }
    _tabWidget->setCurrentIndex(_chosenTabs[index]);
    connect(_tabWidget, SIGNAL(currentChanged(int)), this, SLOT(tabChanged(int)));
    updateUnit(_cmbAngleUnit->currentText().toStdString(), _cmbDistanceUnit->currentText().toStdString());
}

void Mobile::tabChanged(int index) {
    _chosenTabs[_cmbDevices->currentIndex()] = (unsigned int)index;
}

void Mobile::cmbUnitChanged( int index ) {
    updateUnit(_cmbAngleUnit->currentText().toStdString(), _cmbDistanceUnit->currentText().toStdString());
}

/*
 * Update all sliders as well as the property map. If no such property exists, "set" will create it.
 */
void Mobile::updateUnit(const std::string& angles, const std::string& distances) {
    // Create the unit data: a set of <string, double> pairs containing label
    std::pair<std::vector<std::string>, std::vector<double> > sliderUnitDataJoint = makeSliderUnitData(angles, distances, _angleUnitConverters, _distanceUnitConverters, _selectedDevice);
    std::pair<std::vector<std::string>, std::vector<double> > sliderUnitDataCartesian = makeSliderUnitData(angles, distances, _angleUnitConverters, _distanceUnitConverters, 0);
    if(_mobileWidget) {
    	_mobileWidget->setUnits(sliderUnitDataJoint.second, sliderUnitDataJoint.first);
    }
    if(_rwsSettings) {
        _rwsSettings->set<std::string>("AngleUnit", sliderUnitDataCartesian.first.front());
        _rwsSettings->set<std::string>("DistanceUnit", sliderUnitDataCartesian.first.back());
    }
}

void Mobile::removeTabs() {
    while (_tabWidget->count() > 0) {
        QWidget* widget = _tabWidget->widget(0);
        _tabWidget->removeTab(0);
        delete widget;
    }
    _mobileWidget = NULL;
}

void Mobile::constructTabs(PseudoOmniDevice::Ptr device) {
    // Construct mobile tab for device
	_mobileWidget = new MobileWidget();
    std::vector<std::string> titles(device->getDOF());
    for(unsigned int i = 0; i < device->getDOF(); ++i) {
      std::stringstream ss;
      ss << "q" << i;
      titles[i] = ss.str();
    }
    _mobileWidget->setup(titles, device->getBounds(), device->getQ(_state), device->getControlls());

    QPushButton* btnPasteQ = new QPushButton("Paste", _mobileWidget);
    QHBoxLayout* btnlayout = new QHBoxLayout();
    btnlayout->addWidget(new QLabel(""));
    btnlayout->addWidget(btnPasteQ);
    connect(btnPasteQ, SIGNAL(clicked()), _mobileWidget, SLOT(paste()));
    
    QVBoxLayout* tablayout = new QVBoxLayout();
    tablayout->addLayout(btnlayout);
    tablayout->addWidget(_mobileWidget);

    QWidget* tabWidget = new QWidget();
    tabWidget->setLayout(tablayout);

    //_tabWidget->addTab(_jointSliderWidget, "Joint");
    _tabWidget->addTab(tabWidget, "Mobile");
    connect(_mobileWidget, SIGNAL(valueChanged(const rw::math::Q&)), this, SLOT(deviceConfigChanged(const rw::math::Q&)));
    connect(_mobileWidget, SIGNAL(modeChanged(rw::models::PseudoOmniDevice::MODE)), this, SLOT(deviceModeChanged(rw::models::PseudoOmniDevice::MODE)));
    connect(_mobileWidget, SIGNAL(controlsChanged(int,int)), this, SLOT(deviceControlChanged(int,int)));
}

void Mobile::stateChanged(const rw::kinematics::State& state) {
    if (_updating)
        return;

    getRobWorkStudio()->setState(state);
}

void Mobile::deviceConfigChanged(const Q& q) {
    if (_updating)
        return;

    if (_selectedDevice != NULL) {
        _selectedDevice->setQ(q, _state);
        if (_selectedDevice->getMode() == PseudoOmniDevice::TURN) {
        	Vector3D<> icm;
        	if (_selectedDevice->findICM(icm,_state)) {
        		Transform3D<> base = _selectedDevice->getBase()->getTransform(_state);
        		_renderICM->setICM(base.R()*icm+base.P(),(icm-base.P()).norm2(),Vector3D<>::z());
        	} else
            	_renderICM->setInactive();
        } else
        	_renderICM->setInactive();
        getRobWorkStudio()->setState(_state);
    }
}

void Mobile::deviceModeChanged(rw::models::PseudoOmniDevice::MODE mode) {
    if (_updating)
        return;

    if (_selectedDevice != NULL) {
        _selectedDevice->setMode(mode,_state);
        if (mode == PseudoOmniDevice::TURN) {
        	Vector3D<> icm;
        	if (_selectedDevice->findICM(icm,_state)) {
        		Transform3D<> base = _selectedDevice->getBase()->getTransform(_state);
        		_renderICM->setICM(base.R()*icm+base.P(),(icm-base.P()).norm2(),Vector3D<>::z());
        	} else
            	_renderICM->setInactive();
        } else
        	_renderICM->setInactive();
        getRobWorkStudio()->setState(_state);
    }
}

void Mobile::deviceControlChanged(int ind1, int ind2) {
    if (_updating)
        return;

    if (_selectedDevice != NULL) {
        _selectedDevice->setControlls(ind1,ind2,_state);
        getRobWorkStudio()->setState(_state);
    }
}

void Mobile::close()
{
    disconnect(_tabWidget, 0, 0, 0);
    _workcell = NULL;
    _cmbDevices->clear();
    _items.clear();
    _chosenTabs.clear();
    removeTabs();
    _frameToIndex.clear();
    if (_renderICM != NULL)
    	_renderICM = new RenderICM();
}


void Mobile::updateValues() {
    _updating = true;
    if (_selectedDevice != NULL && _mobileWidget != NULL) {
        Q q = _selectedDevice->getQ(_state);
        _mobileWidget->updateValues(q);
    }
    _updating = false;
}


void Mobile::showEvent ( QShowEvent * event )
{
    _state = getRobWorkStudio()->getState();
    updateValues();
}

void Mobile::stateChangedListener(const State& state)
{
    _state = state;
    if (isVisible() && !_updating) {
        updateValues();
    }
}

void Mobile::frameSelectedListener(Frame* frame) {
	if(frame==NULL)
		return;
	if(_frameToIndex.has(*frame)){
		_cmbDevices->setCurrentIndex(_frameToIndex[*frame]);
	}
}

#ifndef RWS_USE_STATIC_LINK_PLUGINS
Q_EXPORT_PLUGIN2(Mobile, Mobile)
#endif
