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

#include <iomanip>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QPushButton>
#include <QInputDialog>
#include <QLabel>
#include <QSlider>
#include <QClipboard>
#include <QApplication>

#include "SliderTab.hpp"

#include <rw/math/Constants.hpp>
#include <rw/math/RPY.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/Joint.hpp>
#include <rw/models/ParallelDevice.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/invkin/IKMetaSolver.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/invkin/ParallelIKSolver.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::invkin;
using namespace rw::proximity;
using namespace std;

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
        box->setKeyboardTracking(false);

        const double step = (high - low) / 400;
        box->setSingleStep(step);

        return box;
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
Slider::Slider(const std::string& title,
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
    _low = low;
    _high = high;

    _lowLabel = makeNumericQLabel(_low);
    _slider = makeHSlider(sliderEnd);
    _highLabel = makeNumericQLabel(_high);
    _box = makeDoubleSpinBox(_low, _high);

    unsigned int col = 0;
    if(!title.empty())
        layout->addWidget(_title, row, col++, Qt::AlignLeft);
    layout->addWidget(_lowLabel, row, col++, Qt::AlignRight);
    layout->addWidget(_lowLabel, row, col++); // own lowLabel
    layout->addWidget(_slider, row, col++); // own _slider
    layout->addWidget(_highLabel, row, col++); // own highLabel
    layout->addWidget(_box, row, col); // own _box

    std::stringstream sstr;
    sstr << "Limits: [" << _low  << ";" << _high << "]" << _desc;
    setToolTip(sstr.str().c_str());

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

Slider::~Slider() {
	// As the layout is not owned by this widget, we have to remove the following widgets explicitly:
    delete _title;
    delete _lowLabel;
    delete _highLabel;
    delete _slider;
    delete _box;
}

void Slider::unitUpdated() {
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
    setToolTip(sstr.str().c_str());
}

void Slider::boxValueChanged(double val)
{
    _boxChanged = true;

    // Change the value of the slider.
    if (!_sliderChanged) {
        setSliderValueFromBox(val);
    }

    _sliderChanged = false;
    emit valueChanged();
}

void Slider::sliderValueChanged(int val)
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

void Slider::enableDisable(int val) {
	if (val == Qt::Checked) {
		_box->setEnabled(true);
		_slider->setEnabled(true);
	} else if (val == Qt::Unchecked) {
		_box->setEnabled(false);
		_slider->setEnabled(false);
	}
}

void Slider::setSliderValueFromBox(double val)
{
//    _slider->setValue((int)((val - _low) / (_high - _low) * sliderEnd));
    _slider->setValue((int)((val/_toUnit - _low) / (_high - _low) * sliderEnd));
}

void Slider::setBoxValueFromSlider(int val)
{
//    _box->setValue(((double)val / sliderEnd) * (_high - _low) + _low);
    _box->setValue( ( ((double)val / sliderEnd) * (_high - _low) + _low ) * _toUnit);
}

double Slider::value() const
{
    return _box->value() / _toUnit;
}

void Slider::setValue(double val)
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

MovableFrameTab::MovableFrameTab(const std::pair<rw::math::Q, rw::math::Q>& bounds,
                           MovableFrame* frame,
                           rw::models::WorkCell* workcell,
                           const rw::kinematics::State& state):
    _state(state),
    _frame(frame),
	_updating(false)
{
    QGridLayout* tablayout = new QGridLayout(this); //owned
    QWidget* toppanel = new QWidget();
    QHBoxLayout* toplayout = new QHBoxLayout(toppanel);

    QWidget* sliderpanel = new QWidget();
    _layout = new QGridLayout(sliderpanel);



    _frames = Kinematics::findAllFrames(workcell->getWorldFrame(), _state);
    _cmbFrames = new QComboBox();
    int i = 0;
    for (std::vector<Frame*>::iterator it = _frames.begin(); it != _frames.end(); ++it, ++i) {
        _cmbFrames->addItem((*it)->getName().c_str(), QVariant(i));
    }
    connect(_cmbFrames, SIGNAL(currentIndexChanged(int)), this, SLOT(refFrameChanged(int)));
    _refframe = workcell->getWorldFrame(); //Setup the reference frame

    toplayout->addWidget(new QLabel("Ref. Frame: "));
    toplayout->addWidget(_cmbFrames);
    
    tablayout->addWidget(toppanel, 0, 0);
    _transformSliderWidget = new TransformSliderWidget(bounds, Kinematics::frameTframe(_refframe, _frame, _state));
    
    QPushButton* btnPasteQ = new QPushButton("Paste", _transformSliderWidget);
    QPushButton* btnCopyQ = new QPushButton("Copy", _transformSliderWidget);
    QHBoxLayout* btnlayout = new QHBoxLayout();
    btnlayout->addWidget(btnCopyQ);
    btnlayout->addWidget(btnPasteQ);
    tablayout->addLayout(btnlayout, 1, 0);
    connect(btnPasteQ, SIGNAL(clicked()), _transformSliderWidget, SLOT(paste()));
    connect(btnCopyQ, SIGNAL(clicked()), _transformSliderWidget, SLOT(copy()));
    
    connect(_transformSliderWidget,
            SIGNAL(valueChanged(const rw::math::Transform3D<>&)),
            this,
            SLOT(transformChanged(const rw::math::Transform3D<>&)));

    tablayout->addWidget(_transformSliderWidget, 2, 0);




}

void MovableFrameTab::setUnits(const std::vector<double> &converters, const std::vector<std::string> &descriptions) {
    _transformSliderWidget->setUnits(converters, descriptions);
}

void MovableFrameTab::refFrameChanged(int index) {
    _refframe = _frames[index];
    doUpdateValues();
}

void MovableFrameTab::updateValues(const State& state) {
    _state = state;
    doUpdateValues();
}

void MovableFrameTab::doUpdateValues() {
	if (_updating)
		return;
    Transform3D<> transform = Kinematics::frameTframe(_refframe, _frame, _state);
    _transformSliderWidget->updateValues(transform);
}


void MovableFrameTab::transformChanged(const Transform3D<>& transform) {
    Transform3D<> parent2ref = Kinematics::frameTframe(_frame->getParent(_state), _refframe, _state);
    Transform3D<> result = parent2ref*transform;
    _frame->setTransform(result, _state);

	_updating = true;
    stateChanged(_state);
	_updating = false;
}



JointSliderWidget::JointSliderWidget():
	_enableAngularCombined(false)
{
    _layout = new QGridLayout(this); // owned
}



void JointSliderWidget::setup(const std::vector<std::string>& titles,
                              const std::pair<Q,Q>& bounds,
                              const Q& q,
							  bool enablers,
							  bool enableAngularCombined) {
	_enableAngularCombined = enableAngularCombined;
  
  // Hack so that we can move the first slider with mouse
    //QLabel* lbl = new QLabel("");
    //_layout->addWidget(lbl, 0,1 ); // own _slider
	
  
  /*
	QPushButton* btnPasteQ = new QPushButton("Paste", this);
	_layout->addWidget(btnPasteQ, 0,0);
	connect(btnPasteQ, SIGNAL(clicked()), this, SLOT(paste()));
  */
    for(std::size_t i = 0; i < _sliders.size(); i++) {
        if(_sliders[i] != NULL) {
            delete _sliders[i];
        }
    }
    for(std::size_t i = 0; i < _enablers.size(); i++) {
        if(_enablers[i] != NULL) {
            delete _enablers[i];
        }
    }
    _sliders.clear();
    _enablers.clear();
    for (size_t i = 0; i < bounds.first.size(); i++) {
        Slider* slider = new Slider(titles[i], bounds.first(i), bounds.second(i), _layout, (int)i+2, this);
        slider->setValue(q(i));
        slider->unitUpdated();
        connect(slider, SIGNAL(valueChanged()), this, SLOT(valueChanged()));
        _sliders.push_back(slider);

        if (enablers) {
        	QCheckBox* enabler = new QCheckBox(this);
        	enabler->setChecked(true);
        	_layout->addWidget(enabler,i+2,6);
        	connect(enabler, SIGNAL(stateChanged(int)), slider, SLOT(enableDisable(int)));
        	if (_enableAngularCombined && i >= 3) {
            	connect(enabler, SIGNAL(stateChanged(int)), slider, SLOT(enableDisable(int)));
        	}
        	_enablers.push_back(enabler);
        }
    }
    //_layout->addWidget(new QLabel(""), (int)bounds.first.size()+2,1 ); // own _slider
	_layout->setRowStretch((int)bounds.first.size()+2, 1);
}



void JointSliderWidget::paste() {

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
		catch (const Exception&) {
			QMessageBox::critical(this, tr("RobWorkStudio Jog"), tr("Unable to parse '%1' as Q").arg(txt));
			continue;
		}
	} while (true);

}

void JointSliderWidget::copy() {
    Q q(_sliders.size());
      
    for(size_t i = 0; i < _sliders.size(); ++i) {
        q[i] = _sliders[i]->value();
    }

    std::stringstream ss;
    ss << q;
    
    QApplication::clipboard()->setText(QString(ss.str().c_str()));
}

void JointSliderWidget::setUnits(const std::vector<double>& converters, const std::vector<std::string>& descriptions) {
    RW_ASSERT(_sliders.size() == converters.size());
    for(size_t i = 0; i < _sliders.size(); ++i) {
        _sliders[i]->setUnitConverter(converters[i]);
        _sliders[i]->setUnitDescription(descriptions[i]);
        _sliders[i]->unitUpdated();
    }

}

void JointSliderWidget::updateValues(const rw::math::Q& q) {
    for (size_t i = 0; i<q.size(); i++) {
        _sliders[i]->setValue(q(i));
    }
}

void JointSliderWidget::updateInactiveValues(const Q& q) {
	for (std::size_t i = 0; i < _enablers.size(); i++) {
		if (!_enablers[i]->isChecked()) {
			_sliders[i]->setValue(q(i));
		}
	}
}

void JointSliderWidget::updateSpecificValue(std::size_t i, double q) {
	_sliders[i]->setValue(q);
}

rw::math::Q JointSliderWidget::getQ() {
    Q q(_sliders.size());
    for (size_t i = 0; i<_sliders.size(); i++) {
        q(i) = _sliders[i]->value();
    }
    return q;
}

std::vector<bool> JointSliderWidget::enabledState() const {
	std::vector<bool> res(_enablers.size());
	for (std::size_t i = 0; i < _enablers.size(); i++)
		res[i] = _enablers[i]->isChecked();
	return res;
}

void JointSliderWidget::valueChanged() {
    valueChanged(getQ());
}

void JointSliderWidget::angularChanged(int state) {
    if (_enableAngularCombined) {
    	_enablers[3]->setChecked(state == Qt::Checked);
    	_enablers[4]->setChecked(state == Qt::Checked);
    	_enablers[5]->setChecked(state == Qt::Checked);
    }
}

namespace {

    Q transform2q(const Transform3D<>& transform) {
        Q q(6);
        RPY<> rpy(transform.R());
        for (size_t i = 0; i<3; i++) {
            q(i) = transform.P()(i);
            q(i+3) = rpy(i);
        }
        return q;
    }

    Transform3D<> q2transform(const Q& q) {
        return Transform3D<>(Vector3D<>(q(0), q(1), q(2)), RPY<>(q(3), q(4), q(5)));
    }

}

TransformSliderWidget::TransformSliderWidget(const std::pair<rw::math::Q, rw::math::Q>& bounds,
		const rw::math::Transform3D<>& transform,
		AngleType angleType,
		bool enablers
):
    _updating(false),
    _angleType(RPYtype),
    _carteasianbounds(bounds),
    _enablers(enablers),
    _lastChangedId(0)
{
    QGridLayout* tablayout = new QGridLayout(this); //owned

    _jointSliderWidget = new JointSliderWidget();
    Q q = transform2q(transform);
    std::vector<std::string> titles(6);
    titles[0] = "x"; titles[1] = "y"; titles[2] = "z";
    titles[3] = "R"; titles[4] = "P"; titles[5] = "Y";

    _jointSliderWidget->setup(titles, bounds, q, enablers, _angleType);
    _last_q = _jointSliderWidget->getQ();
    connect(_jointSliderWidget,
            SIGNAL(valueChanged(const rw::math::Q&)),
            this,
            SLOT(valueChanged(const rw::math::Q&)));
    tablayout->addWidget(_jointSliderWidget, 0, 0);

    if(angleType != RPYtype) {
        _converters = {1,1,1};
        _descriptions = {"meters", "meters", "meters"};
        angleTypeChanged(angleType);
    }
}


void TransformSliderWidget::setUnits(const std::vector<double>& converters, const std::vector<std::string>& descriptions) {
    _converters = converters;
    _descriptions = descriptions;
    if(_angleType == EAAtype) {
        std::vector<double> con = _converters;
        std::vector<std::string> des = _descriptions;
        con.push_back(con[5]);
        des.push_back(des[5]);
        _jointSliderWidget->setUnits(con, des);
    } else if(_angleType == QUAtype) {
        std::vector<double> con;
        std::vector<std::string> des;
        for(std::size_t i = 0; i<3; i++) {
            con.push_back(converters[i]);
            des.push_back(descriptions[i]);
        }
        for(std::size_t i = 3; i<7; i++) {
            con.push_back(1);
            des.push_back("Quaternion");
        }
        _jointSliderWidget->setUnits(con, des);
    } else {
        _jointSliderWidget->setUnits(_converters, _descriptions);
    }

}

Q TransformSliderWidget::qFromTransform(const rw::math::Transform3D<> &transform) {
    Q q(6);
    if (_angleType == QUAtype) {
        q = Q(7);
        Quaternion<double> qua(transform.R());
        for(std::size_t i = 0; i<3; i++){
            q[i] = transform.P()[i];
        }
        if(Quaternion<double>(qua+_QUAfromSliders).getLength() < Quaternion<double>(qua-_QUAfromSliders).getLength()){
            qua = -qua;
        }
        q[3] = qua.getQx();
        q[4] = qua.getQy();
        q[5] = qua.getQz();
        q[6] = qua.getQw();

    } else if(_angleType == EAAtype) {
        q = Q(7);
        EAA<> eaa(transform.R());
        if(_EAAfromSliders.angle() > Pi) {
            eaa = EAA<>(eaa.axis()*(-1), _EAAfromSliders.angle());
        } else if(_EAAfromSliders.angle() > 0) {
            eaa = EAA<>(eaa.axis(), _EAAfromSliders.angle());
        }


        for (std::size_t i = 0; i < 3; i++) {
            q[i] = transform.P()[i];
            q[i+3] = eaa[i];
        }
        q[6] = eaa.angle();
    } else {
        q = transform2q(transform);
    }
    return q;
}
void TransformSliderWidget::updateValues(const Transform3D<>& transform) {
    if (_updating)
        return;
    _updating = true;
    _jointSliderWidget->updateValues(qFromTransform(transform));
    _updating = false;
    _last_q = _jointSliderWidget->getQ();
}

void TransformSliderWidget::updateInactiveValues(const Transform3D<>& transform) {
    if (_updating)
        return;
    _updating = true;
    Q values = _jointSliderWidget->getQ();
    const Q newValues = qFromTransform(transform);
    _jointSliderWidget->updateInactiveValues(newValues);
    switch(_angleType) {
    case QUAtype:
        if (_lastChangedId == 3) {
            _jointSliderWidget->updateSpecificValue(4,newValues[4]);
            _jointSliderWidget->updateSpecificValue(5,newValues[5]);
            _jointSliderWidget->updateSpecificValue(6,newValues[6]);
        } else if (_lastChangedId > 3) {
            _jointSliderWidget->updateSpecificValue(3,newValues[3]);
        }
        break;
    case EAAtype:
        if (_lastChangedId == 6) {
            _jointSliderWidget->updateSpecificValue(3,newValues[3]);
            _jointSliderWidget->updateSpecificValue(4,newValues[4]);
            _jointSliderWidget->updateSpecificValue(5,newValues[5]);
        } else if (_lastChangedId >= 3) {
            _jointSliderWidget->updateSpecificValue(6,newValues[6]);
        }
        break;
    case RPYtype:
        break;
    }
    _updating = false;
    _last_q = _jointSliderWidget->getQ();
}

rw::math::Transform3D<> TransformSliderWidget::getTransform() {
    Q q = _jointSliderWidget->getQ();
    switch(_angleType) {
    case QUAtype:
        return Transform3D<>(Vector3D<>(q[0],q[1],q[2]), Quaternion<double>(q[3],q[4],q[5],q[6]));
    case EAAtype:
        return Transform3D<>(Vector3D<>(q[0],q[1],q[2]), EAA<>(q[3],q[4],q[5]));
    case RPYtype:
        return q2transform(q);
    default:
    	RW_THROW("The type of angle has not been fully implemented.");
    }
}

VectorND<6, bool> TransformSliderWidget::enabledState() const {
	VectorND<6, bool> res;
	const std::vector<bool> state = _jointSliderWidget->enabledState();
	if (state.size() == 0) {
		for (std::size_t i = 0; i < 6; i++)
			res[i] = false;
	} else {
		RW_ASSERT(state.size() == 6);
		for (std::size_t i = 0; i < 6; i++)
			res[i] = state[i];
	}
	return res;
}

TransformSliderWidget::AngleType TransformSliderWidget::toAngleType(const int i) {
    switch(i) {
    case TransformSliderWidget::RPYtype:
    	return TransformSliderWidget::RPYtype;
    case TransformSliderWidget::EAAtype:
    	return TransformSliderWidget::EAAtype;
    case TransformSliderWidget::QUAtype:
    	return TransformSliderWidget::QUAtype;
    default:
    	RW_THROW("The type of angle is not supported!");
    }
}

void TransformSliderWidget::valueChanged(const rw::math::Q& q) {
    if (_updating)
        return;
    Transform3D<> transform;
    switch(_angleType) {
    case QUAtype:
        if(_lastTransformType == QUAtype) {
            std::size_t changed = 0;
            for(std::size_t i= 3; i<7;i++) {
                if(_last_q[i] != q[i]) {
                    changed = i;
                }
            }
            Q q_mod = q;

            if(changed>=3) {
                double length(0);
                for(std::size_t i= 3; i<7;i++){
                    length+= q[i]*q[i];
                }
                double contribution = q[changed]*q[changed];
                double factor = std::sqrt((1-contribution)/(length - contribution));

                if(factor == INFINITY) {
                    int sign = -1*(q_mod[changed]<0)+(q_mod[changed]>0);
                    for(std::size_t i = 3; i < 7; i++) {
                        if(i!= changed) {
                            q_mod[i] = sign*0.1;
                            _last_q[i] = sign*0.1;
                        }
                    }
                    valueChanged(q_mod);
                    return;
                }
                
                for(std::size_t i = 3; i < 7; i++) {
                    if(i != changed){
                        q_mod[i]*= factor;
                    }
                }
            }
            _QUAfromSliders = Quaternion<double>(q_mod[3],q_mod[4],q_mod[5],q_mod[6]);
            transform = Transform3D<>(Vector3D<>(q[0],q[1],q[2]), _QUAfromSliders);
            _lastChangedId = changed;
        } else {
            _lastChangedId = 0;
            _lastTransformType = QUAtype;
            transform = Transform3D<>(Vector3D<>(q[0],q[1],q[2]), Quaternion<double>(q[3],q[4],q[5],q[6]));
        }
        break;
    case EAAtype:
        if(_lastTransformType == EAAtype) {
            std::size_t changed = 0;
            for(std::size_t i= 3; i<7;i++) {
                if(_last_q[i] != q[i]) {
                    changed = i;
                }
            }
            if(changed == 6) {
                EAA<> eaa(q[3],q[4],q[5]);
                _EAAfromSliders = EAA<>(eaa.axis(),q[6]);
                // ########### Make sure values not out of bound ##############
                QPair bounds =getBounds();
                size_t not_zero = 0;
                double largestReduction = 1;
                for(size_t i = 0; i < 3; i++) {
                    if(_EAAfromSliders[i]!=0) not_zero++;
                    double reduction = 1;
                    if(_EAAfromSliders[i] < bounds.first[3+i] ) {
                        reduction = bounds.first[3+i]/_EAAfromSliders[i];
                    } else if(_EAAfromSliders[i] > bounds.second[3+i]) {
                        reduction = bounds.second[3+i]/_EAAfromSliders[i];
                    }
                    if(reduction < largestReduction) largestReduction = reduction;
                }
                if(largestReduction< 1) {
                    if(not_zero == 1) {
                        largestReduction*=0.9999;
                    }
                    for(size_t i = 0; i < _EAAfromSliders.size(); i++) {
                        _EAAfromSliders[i]*= largestReduction;
                    }
                }
                // ########### Hack for propper conversion to transform ##############
                transform = Transform3D<>(Vector3D<>(q[0],q[1],q[2]), _EAAfromSliders);
                if(_EAAfromSliders.angle() > Pi) {
                    eaa = EAA<>(transform.R());
                    eaa= EAA<>(_EAAfromSliders.axis()*-1,eaa.angle());
                    transform = Transform3D<>(Vector3D<>(q[0],q[1],q[2]), eaa);
                }
            } else if(changed >= 3) {
                EAA<> angle(q[3],q[4],q[5]);
                double length = angle.angle();
                while(length > Pi) {
                    length -= Pi;
                }
                if(length < 0) length = 0;
                _EAAfromSliders = angle;
                transform = Transform3D<>(Vector3D<>(q[0],q[1],q[2]), EAA<>(angle.axis(),length));
            } else {
               _EAAfromSliders = EAA<>(q[3],q[4],q[5]);
                transform = Transform3D<>(Vector3D<>(q[0],q[1],q[2]), _EAAfromSliders);
            }
            _lastChangedId = changed;
        } else {
            _lastChangedId = 0;
            _lastTransformType = EAAtype;
            transform = Transform3D<>(Vector3D<>(q[0],q[1],q[2]), EAA<>(q[3],q[4],q[5]));
        }
        break;
    case RPYtype:
        transform = q2transform(q);
        _lastChangedId = 0;
        _lastTransformType = RPYtype;
        break;
    }
    valueChanged(transform);
}

void TransformSliderWidget::angleTypeChanged(int type){
    const AngleType newAngleType = toAngleType(type);
    Q q = getQ(_angleType,newAngleType);
    std::vector<std::string> titles(6);
    titles[0] = "x"; titles[1] = "y"; titles[2] = "z";
    switch(newAngleType) {
    case QUAtype:
        _QUAfromSliders = Quaternion<>();
        titles[3] = "1"; titles[4] = "i"; titles[5] = "j";
        titles.push_back("k");
        break;
    case EAAtype:
        _EAAfromSliders = EAA<>();
        titles[3] = "EAA x"; titles[4] = "EAA y"; titles[5] = "EAA z";
        titles.push_back("Scale");
        break;
    case RPYtype:
        titles[3] = "R"; titles[4] = "P"; titles[5] = "Y";
        break;
    }
    _jointSliderWidget->setup(titles, getBounds(), q, _enablers);
    _last_q = _jointSliderWidget->getQ();
    setUnits(_converters, _descriptions);
}

void TransformSliderWidget::paste() {
  _jointSliderWidget->paste();
}

void TransformSliderWidget::copy() {
  _jointSliderWidget->copy();
}

TransformSliderWidget::QPair TransformSliderWidget::getBounds(){
    switch(_angleType) {
    case QUAtype:
    {
        QPair bounds;
        Q first(7);
        Q second(7);
        bounds.first = first;
        bounds.second = second;
        for(int i =0;i<3;i++) {
            bounds.first[i] = _carteasianbounds.first[i];
            bounds.second[i] = _carteasianbounds.second[i];
        }
        for(int i =3; i < 7; i++) {
            bounds.first[i] = -1;
            bounds.second[i] = 1;
        }

        return bounds;
    }
    case EAAtype:
    {
        vector<double> first = _carteasianbounds.first.toStdVector();
        vector<double> second = _carteasianbounds.second.toStdVector();
        double bound7 = 0;
        for(size_t i = 3; i < 6;i++) bound7 += second[i]*second[i];
        bound7 = std::sqrt(bound7);
        second.push_back(bound7);
        first.push_back(0);
        QPair bounds;
        bounds.first = Q(first);
        bounds.second = Q(second);
        return bounds;
    }
    case RPYtype:
        return _carteasianbounds;
    default:
    	RW_THROW("The type of angle has not been fully implemented.");
    }
}

Q TransformSliderWidget::getQ(const AngleType fromType, const AngleType toType){
    Q q;
    if(fromType != toType) {
        _angleType = fromType;
        Transform3D<> transform = getTransform();
        _angleType = toType;
        q = qFromTransform(transform);
    } else {
        q = _jointSliderWidget->getQ();
    }
    return q;
}

CartesianDeviceTab::CartesianDeviceTab(const std::pair<rw::math::Q, rw::math::Q>& bounds,
									   Device::Ptr device,
                                       WorkCell* workcell,
                                       const rw::kinematics::State& state):
    _state(state),
    _device(device),
    _updating(true)
{
    QGridLayout* tablayout = new QGridLayout(this); //owned
    QWidget* toppanel = new QWidget();
    QGridLayout* toplayout = new QGridLayout(toppanel);



    _frames = Kinematics::findAllFrames(workcell->getWorldFrame(), state);
    _cmbTcpFrame = new QComboBox();
    _cmbRefFrame = new QComboBox();
    int i = 0;
    for (std::vector<Frame*>::iterator it = _frames.begin(); it != _frames.end(); ++it, ++i) {
        _cmbTcpFrame->addItem((*it)->getName().c_str(), QVariant(i));
        _cmbRefFrame->addItem((*it)->getName().c_str(), QVariant(i));
    }
    connect(_cmbTcpFrame, SIGNAL(currentIndexChanged(int)), this, SLOT(tcpFrameChanged(int)));
    connect(_cmbRefFrame, SIGNAL(currentIndexChanged(int)), this, SLOT(refFrameChanged(int)));


    _tcpFrame = device->getEnd();
    _refFrame = workcell->getWorldFrame(); //Setup the reference frame

    _baseTref = FKRange(_device->getBase(), _refFrame, _state);
    _refTtcp = FKRange(_refFrame, _tcpFrame, _state);


    _cmbTcpFrame->setCurrentIndex(_cmbTcpFrame->findText(_tcpFrame->getName().c_str()));
    _cmbRefFrame->setCurrentIndex(_cmbRefFrame->findText(_refFrame->getName().c_str()));
    _cmbAngleType = new QComboBox();
    QStringList text;
    text.append("RPY angle");
    text.append("EAA angle");
    text.append("Quaternions");
    _cmbAngleType->addItems(text);

    toplayout->addWidget(new QLabel("TCP Frame: "), 0,0);
    toplayout->addWidget(_cmbTcpFrame, 0,1);

    toplayout->addWidget(new QLabel("Ref. Frame: "), 1,0);
    toplayout->addWidget(_cmbRefFrame, 1,1);

    toplayout->addWidget(new QLabel("Angle Type: "),2,0);
    toplayout->addWidget(_cmbAngleType,2,1);

    tablayout->addWidget(toppanel, 0, 0);

    const bool enablers = !_device.cast<ParallelDevice>().isNull();
    const TransformSliderWidget::AngleType angleType = TransformSliderWidget::toAngleType(_cmbAngleType->currentIndex());
    _transformSliderWidget = new TransformSliderWidget(bounds, Kinematics::frameTframe(_refFrame, _tcpFrame, _state), angleType, enablers);
    connect(_cmbAngleType, SIGNAL(currentIndexChanged(int)), _transformSliderWidget, SLOT(angleTypeChanged(int)));

    QPushButton* btnPasteQ = new QPushButton("Paste", _transformSliderWidget);
    QPushButton* btnCopyQ = new QPushButton("Copy", _transformSliderWidget);
    QHBoxLayout* btnlayout = new QHBoxLayout();
    btnlayout->addWidget(btnCopyQ);
    btnlayout->addWidget(btnPasteQ);
    tablayout->addLayout(btnlayout, 1, 0);
    connect(btnPasteQ, SIGNAL(clicked()), _transformSliderWidget, SLOT(paste()));    
    connect(btnCopyQ, SIGNAL(clicked()), _transformSliderWidget, SLOT(copy()));    

    connect(_transformSliderWidget,
            SIGNAL(valueChanged(const rw::math::Transform3D<>&)),
            this,
            SLOT(transformChanged(const rw::math::Transform3D<>&)));

    tablayout->addWidget(_transformSliderWidget, 2, 0);


    if (const ParallelDevice::Ptr pdev = _device.cast<ParallelDevice>())
        _iksolver = ownedPtr(new ParallelIKSolver(pdev.get()));
    else
        _iksolver = ownedPtr(new JacobianIKSolver(_device, _tcpFrame, _state));


    _updating = false; 
}

void CartesianDeviceTab::setUnits(const std::vector<double>& converters, const std::vector<std::string>& descriptions) {
    _transformSliderWidget->setUnits(converters, descriptions);
}

void CartesianDeviceTab::tcpFrameChanged(int index) {
    _tcpFrame = _frames[index];
    if (const ParallelDevice::Ptr pdev = _device.cast<ParallelDevice>())
    	_iksolver = ownedPtr(new ParallelIKSolver(pdev.get()));
    else
    	_iksolver = ownedPtr(new JacobianIKSolver(_device, _tcpFrame, _state));
    _refTtcp = FKRange(_refFrame, _tcpFrame, _state);

    doUpdateValues();
}

void CartesianDeviceTab::refFrameChanged(int index) {
    _refFrame = _frames[index];
    _baseTref = FKRange(_device->getBase(), _refFrame, _state);
    _refTtcp = FKRange(_refFrame, _tcpFrame, _state);
    doUpdateValues();
}


#include <rw/math/Jacobian.hpp>

void CartesianDeviceTab::transformChanged(const Transform3D<>& refTtcp) {
    Transform3D<> baseTref = _baseTref.get(_state);
    Transform3D<> baseTtcp = baseTref*refTtcp;

    std::vector<Q> solutions;
    if (const ParallelIKSolver::Ptr psolver = _iksolver.cast<ParallelIKSolver>()) {
    	std::vector<ParallelIKSolver::Target> targets(1);
    	targets[0] = ParallelIKSolver::Target(_tcpFrame,baseTtcp, _transformSliderWidget->enabledState());
        solutions = psolver->solve(targets, _state);
    } else {
        IKMetaSolver metaSolver(_iksolver.get(), _device, (CollisionDetector*)NULL);
        metaSolver.setMaxAttempts(50);
        solutions = metaSolver.solve(baseTtcp, _state);
    }
    if (solutions.empty()) {
        doUpdateValues();
    } else {
        _device->setQ(solutions.front(), _state);
        _updating = true;
        Transform3D<> refTtcp = _refTtcp.get(_state);
        _transformSliderWidget->updateInactiveValues(refTtcp);
        stateChanged(_state);
        _updating = false;
    }
}

void CartesianDeviceTab::updateValues(const rw::kinematics::State& state) {
    _state = state;
    doUpdateValues();
}

void CartesianDeviceTab::doUpdateValues() {
    if (_updating)
        return;
    Transform3D<> refTtcp = _refTtcp.get(_state);
    _transformSliderWidget->updateValues(refTtcp);
}



