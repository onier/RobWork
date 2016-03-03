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

#include "ContactDetectionTestPlugin.hpp"

#include <rws/propertyview/PropertyViewEditor.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/contacts/ContactDetectorData.hpp>
#include <rwsim/contacts/ContactDetectorTracking.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/log/SimulatorLogScope.hpp>
#include <rwsimlibs/gui/log/SimulatorLogWidget.hpp>
#include <rwsimlibs/test/ContactTest.hpp>

#include "ui_ContactDetectionTestPlugin.h"

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rwsim::contacts;
using namespace rwsim::log;
using namespace rwsimlibs::gui;
using namespace rwsimlibs::test;
using namespace rwsimlibs::plugins;

//static const QString STYLE_PROGRESS_COMMON = "QProgressBar {border: 2px solid grey; border-radius: 5px; text-align: center;} ";
//static const QString STYLE_PROGRESS_SUCCESS = STYLE_PROGRESS_COMMON + "QProgressBar::chunk {background: QLinearGradient( x1: 0, y1: 0, x2: 1, y2: 0,stop: 0 #78d,stop: 1 #238 );}";

ContactDetectionTestPlugin::ContactDetectionTestPlugin():
	RobWorkStudioPlugin("ContactDetectionTestPlugin", QIcon(":/rwsimplugin/SimulationIcon.png")),
	_ui(new Ui::ContactDetectionTestPlugin()),
	_inputEditor(new PropertyViewEditor(this)),
	_data(new ContactDetectorData()),
	_tracking(new ContactDetectorTracking()),
	_log(new SimulatorLogScope()),
	_logWidget(NULL)
{
	_ui->setupUi(this);

    _inputEditor->setHeaderVisible(false);
    _ui->parameterBox->layout()->addWidget(_inputEditor);

    connect(_ui->toolBox, SIGNAL(currentChanged(int)), this, SLOT(toolBoxChanged(int)) );

    connect(_inputEditor, SIGNAL(propertyChanged(const std::string&)), this, SLOT(inputChanged()));

    _ui->toolBox->setCurrentWidget(_ui->selectionBox);
    toolBoxChanged(_ui->toolBox->currentIndex());

    connect(_ui->poses, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(setPose(QListWidgetItem*)) );

    connect(_ui->_clear, SIGNAL(clicked()), this, SLOT(buttonClick()) );
    connect(_ui->_detectBtn, SIGNAL(pressed()), this, SLOT(buttonClick()) );
    connect(_ui->_trackBtn, SIGNAL(pressed()), this, SLOT(buttonClick()) );
    connect(_ui->_trackDetectBtn, SIGNAL(pressed()), this, SLOT(buttonClick()) );
    connect(_ui->_verbose, SIGNAL(pressed()), this, SLOT(buttonClick()) );

	connect(_ui->_contacts, SIGNAL(graphicsUpdated()), this, SLOT(updateGraphics()));
	connect(_ui->_autoDetect, SIGNAL(toggled(bool)), this, SLOT(autoDetect(bool)));
    /*QStringList header;
    header.push_back("Name");
    header.push_back("View");
    _ui->results->setHorizontalHeaderLabels(header);
	_ui->results->horizontalHeader()->setResizeMode(QHeaderView::ResizeToContents);
	_ui->results->horizontalHeader()->setStretchLastSection(true);
	*/
}

ContactDetectionTestPlugin::~ContactDetectionTestPlugin() {
	delete _data;
	delete _tracking;
	delete _log;
	delete _ui;
}

void ContactDetectionTestPlugin::initialize() {
	getRobWorkStudio()->stateChangedEvent().add(boost::bind(&ContactDetectionTestPlugin::stateChangedListener, this, _1), this);
}

void ContactDetectionTestPlugin::toolBoxChanged(int index) {
	_ui->selectionBox->setEnabled(false);
	_ui->parameterBox->setEnabled(false);
	_ui->executionBox->setEnabled(false);
	if (_ui->toolBox->widget(index) == _ui->selectionBox) {
		message("Constructing lists of tests...");
		_ui->testList->clear();
		_test = NULL;
		_input = NULL;
		_wc = NULL;
		const std::vector<std::string> tests = ContactTest::Factory::getTests();
		BOOST_FOREACH(const std::string& test, tests) {
			QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(test));
			item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
			_ui->testList->addItem(item);
		}
		message("Please select which test to perform.");
		_ui->selectionBox->setEnabled(true);
	} else if (_ui->toolBox->widget(index) == _ui->parameterBox) {
		if (_ui->testList->currentItem() == NULL) {
			error("No test has been selected.");
			return;
		}

		message("Creating test...");
		const std::string testName = _ui->testList->currentItem()->text().toStdString();
		_test = ContactTest::Factory::getTest(testName);
		if (_test == NULL) {
			error("Could not create the test.");
			return;
		}

		message("Fetching parameters of test...");
		if (_input.isNull())
			_input = _test->getDefaultParameters();
		_inputEditor->setPropertyMap(_input);

		inputChanged(); // load the workcell

		message("Adjust the default parameters if required.");

		_ui->parameterBox->setEnabled(true);
	} else if (_ui->toolBox->widget(index) == _ui->executionBox) {
    	//_ui->progress->setStyleSheet(STYLE_PROGRESS_SUCCESS);
    	//_ui->progress->setValue(0);

		if (_ui->testList->currentItem() == NULL) {
			error("No test has been selected.");
			return;
		}

		if (_test.isNull()) {
			message("Creating test...");
			const std::string testName = _ui->testList->currentItem()->text().toStdString();
			_test = ContactTest::Factory::getTest(testName);
			if (_test == NULL) {
				error("Could not create the test.");
				return;
			}
		}

		if (_input.isNull()) {
			_input = _test->getDefaultParameters();
			inputChanged(); // load the workcell
		}

		_poses = _test->getPoses(*_input);
		_detector = _test->getDetector(*_input);
		_ui->poses->clear();
		int i = 0;
		for (std::map<std::string, State>::const_iterator it = _poses.begin(); it != _poses.end(); it++) {
			_ui->poses->addItem(QString::fromStdString(it->first));
			i++;
		}

	    const GroupNode::Ptr gnode = getRobWorkStudio()->getView()->getSceneViewer()->getWorldNode();
	    const SceneGraph::Ptr graph = getRobWorkStudio()->getView()->getSceneViewer()->getScene();
	    if (_pluginGraphics.isNull()) {
	    	_pluginGraphics = ownedPtr(new GroupNode("ContactDetectionTestPlugin"));
	    	GroupNode::addChild(_pluginGraphics,gnode);
		    _ui->_contacts->showGraphics(_pluginGraphics,graph);
	    }

		message("Test and workcell loaded.");
		_ui->executionBox->setEnabled(true);
	}
}

void ContactDetectionTestPlugin::inputChanged() {
	RW_ASSERT(!_test.isNull());
	message("Updating workcell with new parameters...");

	_wc = _test->getWC(*_input);
	if (_wc.isNull()) {
		error("Could not retrieve a valid workcell from test.");
		return;
	}

    if (!_pluginGraphics.isNull()) {
        const GroupNode::Ptr gnode = getRobWorkStudio()->getView()->getSceneViewer()->getWorldNode();
    	gnode->removeChild(_pluginGraphics);
    	_pluginGraphics = NULL;
	    _ui->_contacts->showGraphics(NULL,NULL);
    }
    getRobWorkStudio()->setWorkcell(_wc);

	message("Workcell updated with new parameters.");
}

void ContactDetectionTestPlugin::setPose(QListWidgetItem* item) {
	const std::string name = item->text().toStdString();
	if (_poses.find(name) != _poses.end()) {
		getRobWorkStudio()->setState(_poses[name]);
	} else {
		QMessageBox::warning(this,"Pose","The pose with given name could not be found.");
	}
}

void ContactDetectionTestPlugin::buttonClick() {
	const QObject* const button = QObject::sender();
	buttonClick(button);
}

void ContactDetectionTestPlugin::updateGraphics() {
	getRobWorkStudio()->getView()->update();
}

void ContactDetectionTestPlugin::autoDetect(bool checked) {
	if (checked) {
		_ui->_detectBtn->setCheckable(true);
		_ui->_trackBtn->setCheckable(true);
		_ui->_trackDetectBtn->setCheckable(true);
	} else {
		_ui->_detectBtn->setChecked(false);
		_ui->_trackBtn->setChecked(false);
		_ui->_trackDetectBtn->setChecked(false);
		_ui->_detectBtn->setCheckable(false);
		_ui->_trackBtn->setCheckable(false);
		_ui->_trackDetectBtn->setCheckable(false);
	}
}

void ContactDetectionTestPlugin::buttonClick(const QObject* button) {
	if (button == _ui->_clear) {
		_ui->_contacts->clearContents();
		_tracking->clear();
		_ui->_clear->setEnabled(false);
		_ui->_trackBtn->setEnabled(false);
		_ui->_trackBtn->setEnabled(false);
		_ui->_trackBtn->setEnabled(false);
	} else if (button == _ui->_detectBtn) {
		if (_detector == NULL) {
			QMessageBox::warning(this,"Detect","Detector not given in test!");
		} else {
			_tracking->clear();
			const std::vector<Contact> contacts = _detector->findContacts(getRobWorkStudio()->getState(),*_data,*_tracking,_log);
			_ui->_contacts->setContacts(contacts);
			_ui->_clear->setEnabled(true);
			_ui->_trackBtn->setEnabled(true);
			_ui->_trackDetectBtn->setEnabled(true);
			_ui->_verbose->setEnabled(true);
		}
		if (_ui->_detectBtn->isCheckable()) {
			_ui->_trackBtn->setChecked(false);
			_ui->_trackDetectBtn->setChecked(false);
			_ui->_contacts->selectAll();
		}
	} else if (button == _ui->_trackBtn) {
		const std::vector<Contact> contacts = _detector->updateContacts(getRobWorkStudio()->getState(),*_data,*_tracking,_log);
		_ui->_contacts->setContacts(contacts);
		if (_ui->_trackBtn->isCheckable()) {
			_ui->_detectBtn->setChecked(false);
			_ui->_trackDetectBtn->setChecked(false);
			_ui->_contacts->selectAll();
		}
	} else if (button == _ui->_trackDetectBtn) {
		QMessageBox::warning(this,"Track and Detect","Use Track instead - not implemented yet.");
		if (_ui->_trackDetectBtn->isCheckable()) {
			_ui->_detectBtn->setChecked(false);
			_ui->_trackBtn->setChecked(false);
			_ui->_contacts->selectAll();
		}
	} else if (button == _ui->_verbose) {
		if (_logWidget == NULL)
			_logWidget = new SimulatorLogWidget(NULL);
		//_logWidget->setDWC(_dwc);
		_logWidget->setLog(_log);
		_logWidget->show();
		_logWidget->raise();
		_logWidget->activateWindow();
	} else {
		QMessageBox::warning(this,"Button Click","Unknown sender of signal.");
	}
}

void ContactDetectionTestPlugin::message(const std::string& msg) {
	_ui->status->setText(QString::fromStdString(msg));
	_ui->status->setStyleSheet("");
}

void ContactDetectionTestPlugin::error(const std::string& msg) {
	_ui->status->setText(QString::fromStdString(msg));
	_ui->status->setStyleSheet("QLabel { color : red; }");
}

void ContactDetectionTestPlugin::stateChangedListener(const State& state) {
	if (_ui->_detectBtn->isChecked()) {
		buttonClick(_ui->_detectBtn);
	} else if (_ui->_trackBtn->isChecked()) {
		buttonClick(_ui->_trackBtn);
	} else if (_ui->_trackDetectBtn->isChecked()) {
		buttonClick(_ui->_trackDetectBtn);
	}
}

#if !RWS_USE_QT5
Q_EXPORT_PLUGIN(ContactDetectionTestPlugin);
#endif
