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

#include "SimulatorStatisticsWidget.hpp"
#include "MathematicaPlotWidget.hpp"

#include <rw/common/macros.hpp>
#include <rwsim/log/SimulatorStatistics.hpp>

#include "ui_SimulatorStatisticsWidget.h"

using namespace rwsim::log;
using namespace rwsimlibs::gui;
#include <boost/foreach.hpp>
SimulatorStatisticsWidget::SimulatorStatisticsWidget(rw::common::Ptr<const SimulatorStatistics> stats, QWidget* parent):
	QWidget(parent),
	_ui(new Ui::SimulatorStatisticsWidget()),
	_stats(stats),
	_mathematica(new MathematicaPlotWidget(this))
{
	_ui->setupUi(this);
	_ui->_image->setLayout(new QGridLayout());
	_ui->_image->layout()->addWidget(_mathematica);

	typedef SimulatorStatistics::DataSeries Data;
	const Data& data = stats->getSeries();
	_ui->_listX->clear();
	_ui->_listY->clear();
	for (Data::const_iterator it = data.begin(); it != data.end(); it++) {
		_ui->_listX->addItem(QString::fromStdString(it->first));
		_ui->_listY->addItem(QString::fromStdString(it->first));
	}
	connect(_ui->_listX->selectionModel(),
			SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
			this, SLOT(changed(const QItemSelection &, const QItemSelection &)));
	connect(_ui->_listY->selectionModel(),
			SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
			this, SLOT(changed(const QItemSelection &, const QItemSelection &)));

}

SimulatorStatisticsWidget::~SimulatorStatisticsWidget() {
}

std::string SimulatorStatisticsWidget::getName() const {
	return "Statistics";
}

void SimulatorStatisticsWidget::changed(const QItemSelection&, const QItemSelection&) {
	typedef SimulatorStatistics::DataSeries Data;
	const QModelIndexList indexesX = _ui->_listX->selectionModel()->selectedIndexes();
	const QModelIndexList indexesY = _ui->_listY->selectionModel()->selectedIndexes();
	const Data& data = _stats->getSeries();
	RW_ASSERT(indexesX.size() == 0 || indexesX.size() == 1);

	std::vector<double> selectedDataX;
	if (indexesX.size() > 0) {
		const QModelIndex& index = indexesX.at(0);
		RW_ASSERT(index.column() == 0);
		const std::string name = index.data().toString().toStdString();
		RW_ASSERT(data.find(name) != data.end());
		selectedDataX = data.find(name)->second;

		// Disable the y entries with wrong dimensions
		for (int row = 0; row < _ui->_listY->count(); row++) {
			QListWidgetItem* const item = _ui->_listY->item(row);
			const std::string name = item->text().toStdString();
			RW_ASSERT(data.find(name) != data.end());
			if (data.find(name)->second.size() != selectedDataX.size())
				item->setFlags(Qt::NoItemFlags);
		}
	} else {
		// Enable all y entries
		for (int row = 0; row < _ui->_listY->count(); row++) {
			QListWidgetItem* const item = _ui->_listY->item(row);
			item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled |Qt::ItemIsDragEnabled);
		}
	}

	Data selectedDataY;
	foreach (const QModelIndex& index, indexesY) {
		RW_ASSERT(index.column() == 0);
		const std::string name = index.data().toString().toStdString();
		RW_ASSERT(data.find(name) != data.end());
		selectedDataY[name] = data.find(name)->second;
	}

	if (selectedDataY.size() > 0) {
		if (selectedDataX.size() == 0) {
			Data::const_iterator it = selectedDataY.begin();
			const std::vector<double> valuesY = it->second;
			// TODO: allow more y-series than one
			std::vector<double> x(valuesY.size());
			for (std::size_t i = 0; i < valuesY.size(); i++)
				x[i] = i;
			_mathematica->listPlot(x,valuesY);
		} else if (selectedDataX.size() > 1) {
			Data::const_iterator it = selectedDataY.begin();
			const std::vector<double> valuesY = it->second;
			// TODO: allow more y-series than one
			RW_ASSERT(selectedDataX.size() == valuesY.size());
			_mathematica->listPlot(selectedDataX,valuesY);
		}
	}
}
