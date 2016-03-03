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

#include "ContactSetWidget.hpp"

#include <rw/graphics/GroupNode.hpp>
#include <rw/graphics/SceneGraph.hpp>
#include <rwsim/contacts/RenderContacts.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/log/LogContactSet.hpp>
#include "ui_ContactSetWidget.h"

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::log;
using namespace rwsimlibs::gui;

ContactSetWidget::ContactSetWidget(rw::common::Ptr<const LogContactSet> entry, QWidget* parent):
	SimulatorLogEntryWidget(parent),
	_ui(new Ui::ContactSetWidget()),
	_contactSet(entry)
{
	_ui->setupUi(this);
	connect(_ui->_contactBodyPairs->selectionModel(),
			SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
			this, SLOT(contactSetPairsChanged(const QItemSelection &, const QItemSelection &)));
	connect(_ui->_contactTable,	SIGNAL(graphicsUpdated()), this, SIGNAL(graphicsUpdated()));

	QStringList headerLabels;
	headerLabels.push_back("First");
	headerLabels.push_back("Second");
	headerLabels.push_back("Contacts");
	_ui->_contactBodyPairs->setHorizontalHeaderLabels(headerLabels);
}

ContactSetWidget::~ContactSetWidget() {
	if (_root != NULL) {
		_root->removeChild("Contacts");
	}
}

void ContactSetWidget::setDWC(rw::common::Ptr<const DynamicWorkCell> dwc) {
}

void ContactSetWidget::setEntry(rw::common::Ptr<const SimulatorLog> entry) {
	const rw::common::Ptr<const LogContactSet> set = entry.cast<const LogContactSet>();
	if (!(set == NULL))
		_contactSet = set;
	else
		RW_THROW("_contactSet (setEntry): invalid entry!");
}

rw::common::Ptr<const SimulatorLog> ContactSetWidget::getEntry() const {
	return _contactSet;
}

void ContactSetWidget::updateEntryWidget() {
	_ui->_contactDescription->setText(QString::fromStdString(_contactSet->getDescription()));
	_ui->_contacts->setText(QString::number(_contactSet->size()));
	typedef std::pair<std::string, std::string> FramePair;
	std::set<FramePair> pairs;
	BOOST_FOREACH(const Contact& c, _contactSet->getContacts()) {
		const std::string& nameA = c.getNameA();
		const std::string& nameB = c.getNameB();
		if (nameA < nameB)
			pairs.insert(std::make_pair(nameA,nameB));
		else
			pairs.insert(std::make_pair(nameB,nameA));
	}
	_ui->_contactBodyPairs->setRowCount(pairs.size());
	int row = 0;
	_ui->_contactBodyPairs->setSortingEnabled(false);
	BOOST_FOREACH(const FramePair& pair, pairs) {
		// Count how many contacts there are for this pair
		int contacts = 0;
		BOOST_FOREACH(const Contact& c, _contactSet->getContacts()) {
			if (c.getNameA() == pair.first && c.getNameB() == pair.second)
				contacts++;
			else if (c.getNameA() == pair.second && c.getNameB() == pair.first)
				contacts++;
		}
		// Note: setItem takes ownership of the QTableWidgetItems
		QTableWidgetItem* const itemA = new QTableWidgetItem(QString::fromStdString(pair.first));
		QTableWidgetItem* const itemB = new QTableWidgetItem(QString::fromStdString(pair.second));
		QTableWidgetItem* const itemC = new QTableWidgetItem(QString::number(contacts));
		_ui->_contactBodyPairs->setItem(row,0,itemA);
		_ui->_contactBodyPairs->setItem(row,1,itemB);
		_ui->_contactBodyPairs->setItem(row,2,itemC);
		row++;
	}
	_ui->_contactBodyPairs->setSortingEnabled(true);
	if (pairs.size() > 0)
		_ui->_contactBodyPairs->setRangeSelected(QTableWidgetSelectionRange(0,0,pairs.size()-1,2),true);
}

void ContactSetWidget::showGraphics(GroupNode::Ptr root, SceneGraph::Ptr graph) {
	if (root == NULL && _root != NULL)
		_root->removeChild("Contacts");
	_root = root;
	_graph = graph;
	GroupNode::Ptr contactGroup = ownedPtr(new GroupNode("Contacts"));
	GroupNode::addChild(contactGroup, _root);
	_ui->_contactTable->showGraphics(contactGroup,graph);
}

std::string ContactSetWidget::getName() const {
	return "Contact Set";
}

void ContactSetWidget::contactSetPairsChanged(const QItemSelection&, const QItemSelection&) {
	typedef std::pair<std::string, std::string> NamePair;
	const QModelIndexList indexes = _ui->_contactBodyPairs->selectionModel()->selectedIndexes();
	std::list<NamePair> names;
	foreach (QModelIndex index, indexes) {
		if (index.column() != 0)
			continue;
		const std::string nameA = _ui->_contactBodyPairs->item(index.row(),0)->data(Qt::DisplayRole).toString().toStdString();
		const std::string nameB = _ui->_contactBodyPairs->item(index.row(),1)->data(Qt::DisplayRole).toString().toStdString();
		names.push_back(NamePair(nameA,nameB));
	}
	std::vector<std::size_t> contactsToShow;
	for (std::size_t i = 0; i < _contactSet->size(); i++) {
		const Contact& c = _contactSet->getContact(i);
		bool show = false;
		BOOST_FOREACH(const NamePair& name, names) {
			const std::string& nameA = c.getNameA();
			const std::string& nameB = c.getNameB();
			if (nameA == name.first && nameB == name.second)
				show = true;
			else if (nameB == name.first && nameA == name.second)
				show = true;
			if (show)
				break;
		}
		if (show)
			contactsToShow.push_back(i);
	}
	std::vector<Contact> contactVec;
	BOOST_FOREACH(const std::size_t i, contactsToShow) {
		const Contact& c = _contactSet->getContact(i);
		contactVec.push_back(c);
	}
	_ui->_contactTable->setContacts(contactVec);
	_ui->_contactTable->selectAll();
}

ContactSetWidget::Dispatcher::Dispatcher() {
}

ContactSetWidget::Dispatcher::~Dispatcher() {
}

SimulatorLogEntryWidget* ContactSetWidget::Dispatcher::makeWidget(rw::common::Ptr<const SimulatorLog> entry, QWidget* parent) const {
	const rw::common::Ptr<const LogContactSet> contactSet = entry.cast<const LogContactSet>();
	if (!(contactSet == NULL))
		return new ContactSetWidget(contactSet, parent);
	RW_THROW("ContactSetWidget::Dispatcher (makeWidget): invalid entry!");
	return NULL;
}

bool ContactSetWidget::Dispatcher::accepts(rw::common::Ptr<const SimulatorLog> entry) const {
	if (!(entry.cast<const LogContactSet>() == NULL))
		return true;
	return false;
}
