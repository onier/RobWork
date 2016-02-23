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

#ifndef MOBILEMODULE_H
#define MOBILEMODULE_H

#include <QTabWidget>
#include <QTextEdit>
#include <QSlider>
#include <QtGui>

#include <list>
#include <vector>

#include <rw/graphics/Render.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/MobileDevice.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/FrameMap.hpp>

#include <rws/RobWorkStudioPlugin.hpp>

#include "RenderICM.hpp"
#include "MobileTab.hpp"

namespace rws {

/**
 * @brief The Mobile plugin enables control of mobile devices by jogging.
 */
class Mobile: public RobWorkStudioPlugin
{
    Q_OBJECT
#ifndef RWS_USE_STATIC_LINK_PLUGINS
    Q_INTERFACES(rws::RobWorkStudioPlugin)
#endif
public:
	/**
	 * @brief Constructor
	 */
    Mobile();

	/**
	 * @brief Destructor
	 */
    virtual ~Mobile();

	/**
	 * @copydoc RobWorkStudioPlugin::initialize
	 */
    virtual void initialize();

	/**
	 * @copydoc RobWorkStudioPlugin::open
	 */
    virtual void open(rw::models::WorkCell* workcell);

	/**
	 * @copydoc RobWorkStudioPlugin::close
	 */
    virtual void close();

	/**
	 * @copydoc RobWorkStudioPlugin::frameSelectedListener
	 */
    void frameSelectedListener(rw::kinematics::Frame* frame);

protected:
    void showEvent ( QShowEvent * event );

private slots:
    void cmbChanged ( int index );
    void cmbUnitChanged(int index);

    void tabChanged(int);

    void stateChanged(const rw::kinematics::State& state);

    void deviceConfigChanged(const rw::math::Q& q);
    void deviceModeChanged(rw::models::PseudoOmniDevice::MODE mode);
    void deviceControlChanged(int ind1, int ind2);

private:

    rw::models::WorkCell* _workcell;
    rw::kinematics::State _state;
	rw::models::PseudoOmniDevice::Ptr _selectedDevice;
    MobileWidget* _mobileWidget;

    bool _updating;

    QComboBox* _cmbDevices;
    QTabWidget* _tabWidget;
	std::vector<rw::models::PseudoOmniDevice::Ptr> _items;
    std::vector<unsigned int> _chosenTabs;
    QComboBox *_cmbAngleUnit, *_cmbDistanceUnit;

    std::pair<rw::math::Q, rw::math::Q> _cartesianBounds;

    void removeTabs();
	void constructTabs(rw::models::PseudoOmniDevice::Ptr device);

    void stateChangedListener(const rw::kinematics::State& state);
    void keyListener(int key, Qt::KeyboardModifiers modifiers);

    QIcon getIcon();
    void updateValues();

    void updateUnit(const std::string& angles, const std::string& distances);
    rw::common::PropertyMap* _rwsSettings;
    std::map<std::string, double> _angleUnitConverters, _distanceUnitConverters;

    rw::kinematics::FrameMap<int> _frameToIndex;

    RenderICM* _renderICM;
};

}

#endif //#ifndef MOBILEMODULE_H
