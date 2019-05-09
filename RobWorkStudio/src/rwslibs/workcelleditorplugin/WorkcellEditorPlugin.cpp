/********************************************************************************
 * Copyright 2018 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "WorkcellEditorPlugin.hpp"

#include <QVBoxLayout>
#include <QMenu>
#include <QToolBar>

#include <rws/RobWorkStudio.hpp>
#include <rwslibs/workcelleditor/WorkcellEditorWindow.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::graphics;
using namespace rw::trajectory;
using namespace rwlibs;
using namespace rws;

WorkcellEditorPlugin::WorkcellEditorPlugin()
    :
    RobWorkStudioPlugin("Workcell Editor", QIcon(":/wceditoricon.png")),
    _editor(NULL)
{
}

WorkcellEditorPlugin::~WorkcellEditorPlugin()
{
}

void WorkcellEditorPlugin::initialize()
{
  QWidget *widget = new QWidget(this);
  QVBoxLayout *lay = new QVBoxLayout(widget);
  widget->setLayout(lay);
  this->setWidget(widget);

  if(_editor==NULL){
    _editor = new WorkcellEditorWindow( RobWorkStudioPlugin::_log , getRobWorkStudio(), this);
  }

  if (_editor->isVisible()) {
    _editor->setVisible(false);
  } else {
    _editor->show();
  }

  lay->addWidget(_editor);
}

void WorkcellEditorPlugin::startEditor()
{
  if(_editor==NULL){
    _editor = new WorkcellEditorWindow(RobWorkStudioPlugin::_log , getRobWorkStudio(), this);
  }

  if (_editor->isVisible()) {
    _editor->setVisible(false);
  } else {
    _editor->show();
  }

}

void WorkcellEditorPlugin::stateChangedListener(const State& state)
{
    _state = state;
}

void WorkcellEditorPlugin::open(WorkCell* workcell)
{
    stateChangedListener(getRobWorkStudio()->getState());
}

void WorkcellEditorPlugin::close()
{
}

void WorkcellEditorPlugin::setupMenu(QMenu* pluginmenu){
    QMenuBar *menu = getRobWorkStudio()->menuBar();

    _openWorkcellEditorAction = new QAction(tr("Workcell Editor"), this); // owned
    connect(_openWorkcellEditorAction, SIGNAL(triggered()), this, SLOT(startEditor()));

    boost::tuple<QWidget*, QMenu*, int> action = RobWorkStudioPlugin::getMenu((QWidget*)menu, std::string("&Tools") );
    if(action.get<1>()!=NULL){
        action.get<1>()->addAction( _openWorkcellEditorAction );
    } else {

    }
}



//----------------------------------------------------------------------
#ifndef RW_STATIC_LINK_PLUGINS
#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN2(WorkcellEditorPlugin, WorkcellEditorPlugin)
#endif
#endif
