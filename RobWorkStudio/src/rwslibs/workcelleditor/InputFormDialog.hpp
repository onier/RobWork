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

#ifndef INPUTFORMDIALOG_HPP
#define INPUTFORMDIALOG_HPP

#include <iterator>
#include <vector>
#include <utility>

#include <QAbstractSpinBox>
#include <QtGlobal>
#include <QString>
#include <QVariant>

namespace InputFormDialog
{

class FormData
{
 public:
  QVariant& operator[](const QString& key);

  std::vector<std::pair<QString, QVariant>>::iterator begin();
  std::vector<std::pair<QString, QVariant>>::iterator end();

  template <typename T>
  T at(const QString& key) const
  {
    auto value = getValue(key);

    Q_ASSERT_X(!value.isNull(), "FormTemplate::at", "invalid key");
    Q_ASSERT_X(value.canConvert<T>(), "FormTemplate::at", "invalid type");

    return value.value<T>();
  }

 private:
  QVariant getValue(const QString& key) const;

  std::vector<std::pair<QString, QVariant>> data_;
};

struct FormOptions
{
  bool listReturnsIndex = false;
  bool listDisplaysAsRadios = false;

  int numericMin = -100;
  int numericMax = 100;
  int numericPrecision = 2;
};

bool getInput(const QString& title, FormData& data, const FormOptions& options = FormOptions());

}

#endif //INPUTFORMDIALOG_HPP
