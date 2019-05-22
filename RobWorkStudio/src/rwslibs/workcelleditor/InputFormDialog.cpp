// Copyright (c) 2017-2018 Serguei Kalentchouk. All rights reserved.
// Use of this source code is governed by an MIT license that can be found in the LICENSE file.

/********************************************************************************
 * Copyright 2019 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "InputFormDialog.hpp"

#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMap>
#include <QPushButton>
#include <QRadioButton>
#include <QSpinBox>
#include <QVector2D>
#include <QVector3D>
#include <QFileDialog>

#include "SetColorButton.hpp"
#include "FileDialogButton.hpp"

namespace InputFormDialog {

    QVariant &
    FormData::operator[](const QString &key) {
        for (auto &pair : data_) {
            if (pair.first == key) {
                return pair.second;
            }
        }

        data_.emplace_back(std::make_pair(key, QVariant()));
        return data_.back().second;
    }

    std::vector<std::pair<QString, QVariant>>::iterator
    FormData::begin() {
        return data_.begin();
    }

    std::vector<std::pair<QString, QVariant>>::iterator
    FormData::end() {
        return data_.end();
    }

    QVariant
    FormData::getValue(const QString &key) const {
        QVariant result;

        for (auto &pair : data_) {
            if (pair.first == key) {
                result = pair.second;
            }
        }

        return result;
    }

    bool
    getInput(const QString &title, FormData &data, const FormOptions &options) {
        auto dialog = new QDialog();
        dialog->setWindowTitle(title);

        auto layout = new QGridLayout(dialog);
        layout->setMargin(2);
        layout->setSpacing(4);

        QMap<QString, QWidget *> widgetMap;

        auto row = 0;
        for (const auto &pair : data) {
            auto label = new QLabel(pair.first + ":");
            layout->addWidget(label, row, 0);

            switch (pair.second.type()) {
                case QVariant::Bool: {
                    auto widget = new QCheckBox();
                    widget->setChecked(pair.second.toBool());
                    layout->addWidget(widget, row, 1);
                    widgetMap[pair.first] = widget;
                    break;
                }
                case QVariant::Color: {
                    auto widget = new SetColorButton();
                    widget->setColor(pair.second.value<QColor>());
                    layout->addWidget(widget, row, 1);
                    widgetMap[pair.first] = widget;
                    break;
                }
                case QVariant::Double: {
                    auto widget = new QDoubleSpinBox();
                    widget->setButtonSymbols(QAbstractSpinBox::NoButtons);
                    widget->setMaximum(double(options.numericMax));
                    widget->setMinimum(double(options.numericMin));
                    widget->setDecimals(options.numericPrecision);
                    widget->setValue(pair.second.toDouble());
                    layout->addWidget(widget, row, 1);
                    widgetMap[pair.first] = widget;
                    break;
                }
                case QVariant::Int: {
                    auto widget = new QSpinBox();
                    widget->setButtonSymbols(QAbstractSpinBox::NoButtons);
                    widget->setMaximum(options.numericMax);
                    widget->setMinimum(options.numericMin);
                    widget->setValue(pair.second.toInt());
                    layout->addWidget(widget, row, 1);
                    widgetMap[pair.first] = widget;
                    break;
                }
                case QVariant::String: {
                    std::size_t cad_string_found = pair.second.toString().toStdString().find("cad_file");
                    if (cad_string_found != std::string::npos) {
                        std::string tmp_str = pair.second.toString().toStdString();
                        std::size_t found = tmp_str.find_last_of(',');
                        std::string search_path = tmp_str.substr(found + 1);
                        auto widget = new FileDialogButton(QString::fromStdString(search_path));
                        layout->addWidget(widget, row, 1);
                        widgetMap[pair.first] = widget;
                    } else {
                        auto widget = new QLineEdit(pair.second.toString());
                        layout->addWidget(widget, row, 1);
                        widgetMap[pair.first] = widget;
                    }
                    break;
                }
                case QVariant::StringList: {
                    if (options.listDisplaysAsRadios) {
                        auto values = pair.second.toStringList();

                        auto widget = new QWidget();
                        auto wlayout = new QHBoxLayout(widget);
                        wlayout->setMargin(2);
                        wlayout->setSpacing(2);

                        auto isChecked = false;
                        for (const auto &value : values) {
                            auto button = new QRadioButton(value);
                            wlayout->addWidget(button);

                            if (!isChecked) {
                                button->setChecked(true);
                                isChecked = true;
                            }
                        }

                        layout->addWidget(widget, row, 1);
                        widgetMap[pair.first] = widget;
                    }
                    else
                    {
                        auto widget = new QComboBox();
                        widget->addItems(pair.second.toStringList());
                        layout->addWidget(widget, row, 1);
                        widgetMap[pair.first] = widget;
                    }
                    break;
                }
                case QVariant::Vector2D: {
                    auto value = pair.second.value<QVector2D>();

                    auto widget = new QWidget();
                    auto wlayout = new QHBoxLayout(widget);
                    wlayout->setMargin(2);
                    wlayout->setSpacing(2);

                    auto xwidget = new QDoubleSpinBox();
                    xwidget->setButtonSymbols(QAbstractSpinBox::NoButtons);
                    xwidget->setMaximum(double(options.numericMax));
                    xwidget->setMinimum(double(options.numericMin));
                    xwidget->setDecimals(options.numericPrecision);
                    xwidget->setValue(value.x());
                    wlayout->addWidget(xwidget);

                    auto ywidget = new QDoubleSpinBox();
                    ywidget->setButtonSymbols(QAbstractSpinBox::NoButtons);
                    ywidget->setMaximum(double(options.numericMax));
                    ywidget->setMinimum(double(options.numericMin));
                    ywidget->setDecimals(options.numericPrecision);
                    ywidget->setValue(value.y());
                    wlayout->addWidget(ywidget);

                    layout->addWidget(widget, row, 1);
                    widgetMap[pair.first] = widget;
                    break;
                }
                case QVariant::Vector3D: {
                    auto value = pair.second.value<QVector3D>();

                    auto widget = new QWidget();
                    auto wlayout = new QHBoxLayout(widget);
                    wlayout->setMargin(2);
                    wlayout->setSpacing(2);

                    auto xwidget = new QDoubleSpinBox();
                    xwidget->setButtonSymbols(QAbstractSpinBox::NoButtons);
                    xwidget->setMaximum(double(options.numericMax));
                    xwidget->setMinimum(double(options.numericMin));
                    xwidget->setDecimals(options.numericPrecision);
                    xwidget->setValue(value.x());
                    wlayout->addWidget(xwidget);

                    auto ywidget = new QDoubleSpinBox();
                    ywidget->setButtonSymbols(QAbstractSpinBox::NoButtons);
                    ywidget->setMaximum(double(options.numericMax));
                    ywidget->setMinimum(double(options.numericMin));
                    ywidget->setDecimals(options.numericPrecision);
                    ywidget->setValue(value.y());
                    wlayout->addWidget(ywidget);

                    auto zwidget = new QDoubleSpinBox();
                    zwidget->setButtonSymbols(QAbstractSpinBox::NoButtons);
                    zwidget->setMaximum(double(options.numericMax));
                    zwidget->setMinimum(double(options.numericMin));
                    zwidget->setDecimals(options.numericPrecision);
                    zwidget->setValue(value.z());
                    wlayout->addWidget(zwidget);

                    layout->addWidget(widget, row, 1);
                    widgetMap[pair.first] = widget;
                    break;
                }
                default:
                    break;
            }

            row++;
        }

        auto btnLayout = new QHBoxLayout();
        btnLayout->setMargin(2);
        btnLayout->setSpacing(4);
        layout->addLayout(btnLayout, row, 0, 1, 2);

        auto ok = new QPushButton("Ok", dialog);
        ok->setDefault(true);
        QObject::connect(ok, &QPushButton::clicked, dialog, &QDialog::accept);
        btnLayout->addWidget(ok);

        auto cancel = new QPushButton("Cancel", dialog);
        QObject::connect(cancel, &QPushButton::clicked, dialog, &QDialog::reject);
        btnLayout->addWidget(cancel);

        if (dialog->exec() != QDialog::Accepted) {
            dialog->deleteLater();
            return false;
        }

        for (auto &pair : data) {
            switch (pair.second.type()) {
                case QVariant::Bool: {
                    const auto widget = qobject_cast<QCheckBox *>(widgetMap[pair.first]);
                    pair.second = widget->isChecked();
                    break;
                }
                case QVariant::Color: {
                    const auto widget = qobject_cast<SetColorButton *>(widgetMap[pair.first]);
                    pair.second = widget->color();
                    break;
                }
                case QVariant::Double: {
                    const auto widget = qobject_cast<QDoubleSpinBox *>(widgetMap[pair.first]);
                    pair.second = widget->value();
                    break;
                }
                case QVariant::Int: {
                    const auto widget = qobject_cast<QSpinBox *>(widgetMap[pair.first]);
                    pair.second = widget->value();
                    break;
                }
                case QVariant::String: {
                    std::size_t cad_string_found = pair.second.toString().toStdString().find("cad_file");
                    if (cad_string_found != std::string::npos) {
                        const auto widget = qobject_cast<FileDialogButton *>(widgetMap[pair.first]);
                        pair.second = widget->getFilename();
                    } else {
                        const auto widget = qobject_cast<QLineEdit *>(widgetMap[pair.first]);
                        pair.second = widget->text();
                    }

                    break;
                }
                case QVariant::StringList: {
                    if (options.listDisplaysAsRadios) {
                        const auto children = widgetMap[pair.first]->children();

                        auto index = 0;
                        for (const auto &child : children) {
                            const auto widget = qobject_cast<QRadioButton *>(child);
                            if (!widget) continue;

                            if (widget->isChecked()) {
                                if (options.listReturnsIndex) {
                                    pair.second = index;
                                } else {
                                    pair.second = widget->text();
                                }
                            }

                            index++;
                        }
                    } else {
                        const auto widget = qobject_cast<QComboBox *>(widgetMap[pair.first]);
                        if (options.listReturnsIndex) {
                            pair.second = widget->currentIndex();
                        }
                        else
                        {
                            pair.second = widget->currentText();
                        }
                    }
                    break;
                }
                case QVariant::Vector2D: {
                    const auto children = widgetMap[pair.first]->children();
                    const auto xwidget = qobject_cast<QDoubleSpinBox *>(children.at(1));
                    const auto ywidget = qobject_cast<QDoubleSpinBox *>(children.at(2));
                    pair.second = QVector2D(xwidget->value(), ywidget->value());
                    break;
                }
                case QVariant::Vector3D: {
                    const auto children = widgetMap[pair.first]->children();
                    const auto xwidget = qobject_cast<QDoubleSpinBox *>(children.at(1));
                    const auto ywidget = qobject_cast<QDoubleSpinBox *>(children.at(2));
                    const auto zwidget = qobject_cast<QDoubleSpinBox *>(children.at(3));
                    pair.second = QVector3D(xwidget->value(), ywidget->value(), zwidget->value());
                    break;
                }
                default:
                    break;
            }
        }

        dialog->deleteLater();
        return true;
    }

}