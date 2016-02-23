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
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsimlibs/test/ContactTest.hpp>

#include "ui_ContactDetectionTestPlugin.h"

using namespace rwsimlibs::test;
using namespace rwsimlibs::plugins;

static const QString STYLE_PROGRESS_COMMON = "QProgressBar {border: 2px solid grey; border-radius: 5px; text-align: center;} ";
static const QString STYLE_PROGRESS_SUCCESS = STYLE_PROGRESS_COMMON + "QProgressBar::chunk {background: QLinearGradient( x1: 0, y1: 0, x2: 1, y2: 0,stop: 0 #78d,stop: 1 #238 );}";

ContactDetectionTestPlugin::ContactDetectionTestPlugin():
	RobWorkStudioPlugin("ContactDetectionTestPlugin", QIcon(":/rwsimplugin/SimulationIcon.png")),
	_ui(new Ui::ContactDetectionTestPlugin()),
	_inputEditor(new PropertyViewEditor(this))
{
	_ui->setupUi(this);

    _inputEditor->setHeaderVisible(false);
    _ui->parameterBox->layout()->addWidget(_inputEditor);

    connect(_ui->toolBox, SIGNAL(currentChanged(int)), this, SLOT(toolBoxChanged(int)) );

    connect(_inputEditor, SIGNAL(propertyChanged(const std::string&)), this, SLOT(inputChanged()));

    _ui->toolBox->setCurrentWidget(_ui->selectionBox);
    toolBoxChanged(_ui->toolBox->currentIndex());

    QStringList header;
    header.push_back("Name");
    header.push_back("View");
    _ui->results->setHorizontalHeaderLabels(header);
	_ui->results->horizontalHeader()->setResizeMode(QHeaderView::ResizeToContents);
	_ui->results->horizontalHeader()->setStretchLastSection(true);
}

ContactDetectionTestPlugin::~ContactDetectionTestPlugin() {
	delete _ui;
}

void ContactDetectionTestPlugin::initialize() {
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
    	_ui->progress->setStyleSheet(STYLE_PROGRESS_SUCCESS);
    	_ui->progress->setValue(0);

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

    getRobWorkStudio()->setWorkcell(_wc);

	message("Workcell updated with new parameters.");
}

void ContactDetectionTestPlugin::message(const std::string& msg) {
	_ui->status->setText(QString::fromStdString(msg));
	_ui->status->setStyleSheet("");
}

void ContactDetectionTestPlugin::error(const std::string& msg) {
	_ui->status->setText(QString::fromStdString(msg));
	_ui->status->setStyleSheet("QLabel { color : red; }");
}

#if !RWS_USE_QT5
Q_EXPORT_PLUGIN(ContactDetectionTestPlugin);
#endif
