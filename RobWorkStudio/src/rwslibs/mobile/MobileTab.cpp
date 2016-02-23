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

#include <iomanip>
#include <float.h>

#include <QVBoxLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QPushButton>
#include <QInputDialog>
#include <QLabel>
#include <QSlider>
#include <QDial>

#include "MobileTab.hpp"

#include <rw/math/RPY.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/Joint.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::invkin;
using namespace rw::proximity;

namespace
{
    QLabel* makeNumericQLabel(double val)
    {
        std::stringstream s; s << std::setprecision(4) << val;
        return new QLabel(s.str().c_str());
    }

    QSlider* makeHSlider(int end)
    {
        QSlider* slider = new QSlider(Qt::Horizontal);

        slider->setRange(0, end);

        // The normalized step size for Page Up / Page Down keys.
        const double pageScale = 0.10;
        const int pageStep = (int)
            (pageScale *
             (slider->maximum() -
              slider->minimum()));
        slider->setPageStep(pageStep);

        return slider;
    }

    QDoubleSpinBox* makeDoubleSpinBox(double low, double high)
    {
        QDoubleSpinBox* box = new QDoubleSpinBox();
        box->setDecimals(3);
        box->setRange(low, high);

        const double step = (high - low) / 400;
        box->setSingleStep(step);

        return box;
    }

    QDial* makeCircularSlider(int end)
    {
        QDial* slider = new QDial();

        slider->setRange(0, end);

        // The normalized step size for Page Up / Page Down keys.
        const double pageScale = 0.10;
        const int pageStep = (int)
            (pageScale *
             (slider->maximum() -
              slider->minimum()));
        slider->setPageStep(pageStep);

        return slider;
    }

    const int sliderEnd = 4000;
}

//----------------------------------------------------------------------
// JointLine

/*
  A lot of things are missing here:

  Currently we have:

  - lower limit
  - upper limit

  (except this doesn't work for dependent joint limits).

  To make the interface nicer we also need access to:

  - name of joint
  - type of joint
  - joint index (maybe)

  In other words, we need access to the joint frame itself.
*/
SpeedSlider::SpeedSlider(const std::string& title,
               double low,
               double high,
               QGridLayout* layout,
               int row,
               QWidget* parent):
    QWidget(parent),
    _boxChanged(false),
    _sliderChanged(false),
    _toUnit(1.0)
{
	_title = new QLabel(tr(title.data()));
	bool bounded = low > -DBL_MAX && high < DBL_MAX;
	if (bounded) {
		_low = low;
		_high = high;
	} else {
		_low = -Pi;
		_high = Pi;
	}

    _lowLabel = makeNumericQLabel(_low);
    if (bounded)
    	_slider = makeHSlider(sliderEnd);
    else
    	_slider = makeCircularSlider(sliderEnd);
    _highLabel = makeNumericQLabel(_high);
    _box = makeDoubleSpinBox(_low, _high);

    unsigned int col = 0;
    if(!title.empty())
        layout->addWidget(_title, row, col++, Qt::AlignLeft);
    if (bounded) {
    	layout->addWidget(_lowLabel, row, col++, Qt::AlignRight);
    	layout->addWidget(_lowLabel, row, col++); // own lowLabel
    }
    layout->addWidget(_slider, row, col++); // own _slider
    if (bounded) {
    	layout->addWidget(_highLabel, row, col++); // own highLabel
    }
    layout->addWidget(_box, row, col); // own _box
    /*
    layout->addWidget(_lowLabel, row, 0, Qt::AlignRight); // own lowLabel
    layout->addWidget(_lowLabel, row, 0); // own lowLabel
    layout->addWidget(_slider, row, 1); // own _slider
    layout->addWidget(_highLabel, row, 2); // own highLabel
    layout->addWidget(_box, row, 3); // own _box
    */

    std::stringstream sstr;
    sstr << "Limits: [" << _low  << ";" << _high << "]" << _desc;
    this->setToolTip(sstr.str().c_str());

    connect(_box,
            SIGNAL(valueChanged(double)),
            this,
            SLOT(boxValueChanged(double)));

    connect(_slider,
            SIGNAL(valueChanged(int)),
            this,
            SLOT(sliderValueChanged(int)));

    setValue((_low + _high) / 2);
}

void SpeedSlider::unitUpdated() {
    const double lowUnit = _toUnit * _low;
    const double highUnit = _toUnit * _high;

    _lowLabel->setText(QString::number(lowUnit, 'g', 4));
    _highLabel->setText(QString::number(highUnit, 'g', 4));

    disconnect(_box, 0, 0, 0);
    disconnect(_slider, 0, 0, 0);

    const int val = _slider->value();
    _box->setRange(lowUnit, highUnit);
    const double step = (highUnit - lowUnit) / 400;
    _box->setSingleStep(step);
    setBoxValueFromSlider(val);

    connect(_box,
            SIGNAL(valueChanged(double)),
            this,
            SLOT(boxValueChanged(double)));

    connect(_slider,
            SIGNAL(valueChanged(int)),
            this,
            SLOT(sliderValueChanged(int)));

    std::stringstream sstr;
    sstr << "Limits: [" << lowUnit  << ";" << highUnit << "]" << _desc;
    this->setToolTip(sstr.str().c_str());
}

void SpeedSlider::boxValueChanged(double val)
{
    _boxChanged = true;

    // Change the value of the slider.
    if (!_sliderChanged) {
        setSliderValueFromBox(val);
    }

    _sliderChanged = false;
    emit valueChanged();
}

void SpeedSlider::sliderValueChanged(int val)
{
    _sliderChanged = true;

    // Change the value of the box.
    double boxVal = _box->value();
    int boxVali = ((boxVal/_toUnit - _low) / (_high - _low) * sliderEnd );
    bool isBoxSame = boxVali==val;
    if (!isBoxSame) {
        setBoxValueFromSlider(val);
    }

    _boxChanged = false;

//    emit valueChanged();
}

void SpeedSlider::setSliderValueFromBox(double val)
{
//    _slider->setValue((int)((val - _low) / (_high - _low) * sliderEnd));
    _slider->setValue((int)((val/_toUnit - _low) / (_high - _low) * sliderEnd));
}

void SpeedSlider::setBoxValueFromSlider(int val)
{
//    _box->setValue(((double)val / sliderEnd) * (_high - _low) + _low);
    _box->setValue( ( ((double)val / sliderEnd) * (_high - _low) + _low ) * _toUnit);
}

double SpeedSlider::value() const
{
    return _box->value() / _toUnit;
}

void SpeedSlider::setValue(double val)
{
    _sliderChanged = true;
    _boxChanged = true;

    if (_low-0.00001 <= val && val <= _high+0.00001) {
        _box->setValue(val*_toUnit);


        _slider->setValue(
            (int)((val - _low) / (_high - _low) * sliderEnd));

    } else {
        RW_WARN(
            "Jog joint value "
            << val
            << " out of range ["
            << _low
            << ", "
            << _high
            << "].");
    }

    _sliderChanged = false;
    _boxChanged = false;
}


MobileWidget::MobileWidget() {
    _layout = new QGridLayout(this); // owned
}



void MobileWidget::setup(const std::vector<std::string>& titles,
                              const std::pair<Q,Q>& bounds,
                              const Q& q,
                              const std::vector<std::string>& controllable) {
  
  // Hack so that we can move the first slider with mouse
	QLabel* lbl = new QLabel("");
	_layout->addWidget(lbl, 0,1 ); // own _slider
	
  
  /*
	QPushButton* btnPasteQ = new QPushButton("Paste", this);
	_layout->addWidget(btnPasteQ, 0,0);
	connect(btnPasteQ, SIGNAL(clicked()), this, SLOT(paste()));
  */

	QGroupBox* group = new QGroupBox("Mode");
	_stopped = new QRadioButton("Stopped");
	_straight = new QRadioButton("Straight");
	_turn = new QRadioButton("Turn");
	QHBoxLayout *vbox = new QHBoxLayout;
	vbox->addWidget(_stopped);
	vbox->addWidget(_straight);
	vbox->addWidget(_turn);
	group->setLayout(vbox);
	_stopped->setChecked(true);
    _modeSelector = group;
	_layout->addWidget(_modeSelector, 1,0,1,3 );
	connect(_stopped, SIGNAL(toggled(bool)), this, SLOT(modeChanged(bool)));
	connect(_straight, SIGNAL(toggled(bool)), this, SLOT(modeChanged(bool)));
	connect(_turn, SIGNAL(toggled(bool)), this, SLOT(modeChanged(bool)));

	_control1 = new QComboBox();
	_control2 = new QComboBox();
	_control1->hide();
	_control2->hide();
	_controllable = controllable;
	for (std::size_t i = 0; i < controllable.size(); i++) {
		if (i != 1)
			_control1->addItem(QString::fromStdString(controllable[i]),QVariant((int)i));
		if (i != 0)
			_control2->addItem(QString::fromStdString(controllable[i]),QVariant((int)i));
	}

	QHBoxLayout* hbox1 = new QHBoxLayout();
	_controlLabel1 = new QLabel(tr("Control Wheel 1:"));
	_controlLabel1->hide();
	hbox1->addWidget(_controlLabel1);
	hbox1->addWidget(_control1);
	_layout->addLayout(hbox1, 2,0,1,3);
	QHBoxLayout* hbox2 = new QHBoxLayout();
	_controlLabel2 = new QLabel(tr("Control Wheel 2:"));
	_controlLabel2->hide();
	hbox2->addWidget(_controlLabel2);
	hbox2->addWidget(_control2);
	_layout->addLayout(hbox2, 3,0,1,3);

	connect(_control1, SIGNAL(currentIndexChanged(int)), this, SLOT(controlsChanged ( int )));
	connect(_control2, SIGNAL(currentIndexChanged(int)), this, SLOT(controlsChanged ( int )));

	for (size_t i = 0; i<bounds.first.size(); i++) {
		SpeedSlider* slider = new SpeedSlider(titles[i], bounds.first(i), bounds.second(i), _layout, i+4, this);
		slider->setValue(q(i));
		connect(slider, SIGNAL(valueChanged()), this, SLOT(valueChanged()));
		_sliders.push_back(slider);
	}
	_layout->addWidget(new QLabel(""), bounds.first.size()+2,1 ); // own _slider
	_layout->setRowStretch(bounds.first.size()+2, 1);
}



void MobileWidget::paste() {

	QString txt = "";
	do {
		txt = QInputDialog::getText(this, tr("RobWorkStudio Jog"), tr("Paste Q"), QLineEdit::Normal, txt);
		if (txt.isEmpty())
			return;

		std::istringstream sstr;
		sstr.str(txt.toStdString());

		try {
			Q q;
			sstr >> q;
			
			if (q.size() != _sliders.size()) {
				QMessageBox::critical(this, tr("RobWorkStudio Jog"), tr("Number of elements does not match device!"));
				continue;
			}
      
      for(unsigned int i = 0; i < _sliders.size(); ++i)
        q[i] /= _sliders[i]->getUnitConverter();
        
			updateValues(q);
			return;
		}
		catch (const Exception& exp) {
			QMessageBox::critical(this, tr("RobWorkStudio Jog"), tr("Unable to parse '%1' as Q").arg(txt));
			continue;
		}
	} while (true);

}

void MobileWidget::setUnits(const std::vector<double>& converters, const std::vector<std::string>& descriptions) {
    RW_ASSERT(_sliders.size() == converters.size());
    for(size_t i = 0; i < _sliders.size(); ++i) {
        _sliders[i]->setUnitConverter(converters[i]);
        _sliders[i]->setUnitDescription(descriptions[i]);
        _sliders[i]->unitUpdated();
    }

}

void MobileWidget::updateValues(const rw::math::Q& q) {
    for (size_t i = 0; i<q.size(); i++) {
        _sliders[i]->setValue(q(i));
    }
}

rw::math::Q MobileWidget::getQ() {
    Q q(_sliders.size());
    for (size_t i = 0; i<_sliders.size(); i++) {
        q(i) = _sliders[i]->value();
    }
    return q;
}


void MobileWidget::valueChanged() {
    valueChanged(getQ());
}

void MobileWidget::modeChanged(bool on) {
	if (on) {
		if (_stopped->isChecked()) {
			modeChanged(PseudoOmniDevice::STOPPED);
			_controlLabel1->hide();
			_controlLabel2->hide();
			_control1->hide();
			_control2->hide();
		} else if (_straight->isChecked()) {
			modeChanged(PseudoOmniDevice::STRAIGHT);
			_controlLabel1->hide();
			_controlLabel2->hide();
			_control1->hide();
			_control2->hide();
		} else if (_turn->isChecked()) {
			modeChanged(PseudoOmniDevice::TURN);
			_controlLabel1->show();
			_controlLabel2->show();
			_control1->show();
			_control2->show();
		}
	}
}

void MobileWidget::controlsChanged(int index) {
	disconnect(_control1, SIGNAL(currentIndexChanged(int)), this, SLOT(controlsChanged ( int )));
	disconnect(_control2, SIGNAL(currentIndexChanged(int)), this, SLOT(controlsChanged ( int )));
	int ind1 = (int)_control1->itemData(_control1->currentIndex()).toInt();
	int ind2 = (int)_control2->itemData(_control2->currentIndex()).toInt();
	_control1->clear();
	_control2->clear();
	for (std::size_t i = 0; i < _controllable.size(); i++) {
		if ((int)i != ind2)
			_control1->addItem(QString::fromStdString(_controllable[i]),QVariant((int)i));
		if ((int)i != ind1)
			_control2->addItem(QString::fromStdString(_controllable[i]),QVariant((int)i));
	}
	if (ind1 > ind2)
		_control1->setCurrentIndex(ind1-1);
	else
		_control1->setCurrentIndex(ind1);
	if (ind2 > ind1)
		_control2->setCurrentIndex(ind2-1);
	else
		_control2->setCurrentIndex(ind2);
	controlsChanged(ind1,ind2);
	connect(_control1, SIGNAL(currentIndexChanged(int)), this, SLOT(controlsChanged ( int )));
	connect(_control2, SIGNAL(currentIndexChanged(int)), this, SLOT(controlsChanged ( int )));
}
