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

#include "FileDialogButton.hpp"
#include <QFileDialog>
#include <rw/common/StringUtil.hpp>

using namespace rw::common;

FileDialogButton::FileDialogButton(QString path) : QPushButton()
{
    setText("Select CAD File");
    _path = path;
    connect(this, SIGNAL(clicked()), this, SLOT(selectFile()));
}

void FileDialogButton::selectFile()
{
    QString fullPath = QFileDialog::getOpenFileName(this, tr("Select CAD file"), _path, tr("CAD Files (*.stl *.obj)"));
    std::string file_name = StringUtil::getFileName(fullPath.toStdString());
    std::string wcDir = _path.toStdString()+"/";
    std::string relFilePath =
        StringUtil::getRelativeDirectoryName(fullPath.toStdString(), StringUtil::getDirectoryName(wcDir));
    std::string finalPath = relFilePath+file_name;
    if(relFilePath.empty())
        _filename = fullPath;
    else
        _filename = QString::fromStdString(finalPath);
}

const QString& FileDialogButton::getFilename() const
{
    return _filename;
}
