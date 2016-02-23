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

#ifndef MOBILETAB_H
#define MOBILETAB_H

#include <QWidget>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QGridLayout>
#include <QComboBox>
#include <QLabel>
#include <QGroupBox>
#include <QRadioButton>

#include <rw/kinematics/State.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/PseudoOmniDevice.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/FKRange.hpp>
#include <rw/math/Q.hpp>

#include <rw/common/Ptr.hpp>
#include <rw/invkin/IKMetaSolver.hpp>


// The widget for a single joint.
class SpeedSlider : public QWidget
{
    Q_OBJECT

public:
    SpeedSlider(const std::string& title,
           double low,
           double high,
           QGridLayout* layout,
           int row,
           QWidget* parent);

    void unitUpdated();

    // The current value of the joint.
    double value() const;

    // Set a value for the joint.
    void setValue(double val);

    void setUnitConverter(double converter){
        _toUnit = converter;
    }

    double getUnitConverter() const {
        return _toUnit;
    }

    void setUnitDescription(const std::string& str){
        _desc = str;
    }

    void showEndLabel(bool enabled){
        _endlabelEnabled = enabled;
    }

private slots:
    void boxValueChanged(double val);
    void sliderValueChanged(int val);

signals:
    // Emitted whenever the joint value changes.
    void valueChanged();

private:
    void setSliderValueFromBox(double val);
    void setBoxValueFromSlider(int val);

    double _low;
    double _high;

    QAbstractSlider* _slider;
    QDoubleSpinBox* _box;

    bool _boxChanged;
    bool _sliderChanged;

    QLabel *_title, *_lowLabel, *_highLabel;

    double _toUnit;
    std::string _desc;

    bool _endlabelEnabled;
};


class MobileWidget: public QWidget {
  Q_OBJECT

public:
	MobileWidget();

    void setup(const std::vector<std::string>& titles,
               const std::pair<rw::math::Q,rw::math::Q>& bounds,
               const rw::math::Q& q,
               const std::vector<std::string>& controllable);

    void setUnits(const std::vector<double>& converters, const std::vector<std::string>& descriptions);

    void updateValues(const rw::math::Q& q);

    rw::math::Q getQ();

signals:
	void valueChanged(const rw::math::Q& q);
	void modeChanged(rw::models::PseudoOmniDevice::MODE mode);
	void controlsChanged(int ind1, int ind2);
    
public slots:
    void paste();

private slots:
	void valueChanged();
	void modeChanged(bool on);
	void controlsChanged(int index);
    
private:
    std::vector<SpeedSlider*> _sliders;
    QGroupBox* _modeSelector;
    QRadioButton *_stopped, *_straight, *_turn;
    QComboBox *_control1, *_control2;
    QLabel *_controlLabel1, *_controlLabel2;
    std::vector<std::string> _controllable;

    QGridLayout* _layout;
};

#endif //#ifndef MOBILEETAB_H
