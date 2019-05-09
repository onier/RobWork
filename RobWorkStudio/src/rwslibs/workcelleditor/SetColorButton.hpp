// Copyright (c) 2018 Serguei Kalentchouk. All rights reserved.
// Use of this source code is governed by an MIT license that can be found in the LICENSE file.

#ifndef SETCOLORBUTTON_HPP
#define SETCOLORBUTTON_HPP

#include <QColor>
#include <QPushButton>

class SetColorButton : public QPushButton
{
  Q_OBJECT

 public:
  SetColorButton();

  void setColor(const QColor& color);
  const QColor& color() const;

 public slots:
      void changeColor();

 private:
  QColor color_;
};

#endif //SETCOLORBUTTON_HPP
