/********************************************************************************
 * Copyright 2018 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef WORKCELLEDITORWINDOW_HPP_
#define WORKCELLEDITORWINDOW_HPP_

#include <QMainWindow>
#include <rw/common/PropertyMap.hpp>
#include <map>

namespace rw { namespace common { class Log; } }
namespace rwlibs { namespace swig { class LuaState; } }

class WCCodeEditor;
class WorkcellHighlighter;

namespace Ui {
    class WorkcellEditorWindow;
}

class TreeModelCompleter;
class QCompleter;
class QAbstractItemModel;

namespace rws {
	class RobWorkStudio;

    /**
     * @brief A workcell editor that enables editing of workcells within robworkstudio.
     */
    class WorkcellEditorWindow: public QMainWindow {
        Q_OBJECT
    public:

        /**
         * @brief Constructor
         * @param output [in] the log on which to stream print functionality and errors
         * @param rwstudio [in] instance of RobWorkStudio
         * @param parent [in] the Qt parent widget
         */
        WorkcellEditorWindow(rw::common::Ptr<rw::common::Log> output, rws::RobWorkStudio* rwstudio, QWidget *parent);

        //! @brief destructor
        virtual ~WorkcellEditorWindow();

    public slots:
        void on_actionNew_triggered(bool);
        void on_actionOpen_triggered(bool);
        void on_actionSave_triggered(bool);
        void on_actionSave_As_triggered(bool);
        void on_actionRun_triggered(bool);
        void on_actionReload_triggered(bool);
        void on_actionClose_triggered(bool);
        void on_actionAdd_Frame_triggered(bool);
        void on_actionAdd_Drawable_triggered(bool);

        void textChanged();
        void runFinished();
        void ShowContextMenu(const QPoint& p);
        void setCheckAction(QAction*);

    private:
        QAbstractItemModel *modelFromFile(const QString& fileName, TreeModelCompleter* completer);

        struct EditorTab {
        	typedef rw::common::Ptr<EditorTab> Ptr;
        	std::string _id;
            WCCodeEditor *_editor;
            WorkcellHighlighter *_highlighter;
            TreeModelCompleter *_completer;
            std::string _filename;
        };

        EditorTab::Ptr makeEditor();

        bool save();
        bool saveAs();
        bool save(const std::string& filename);
        EditorTab::Ptr getCurrentTab();

    private:
        //! hold
        std::map<QWidget*, EditorTab::Ptr> _editors;
        class Ui::WorkcellEditorWindow *_ui;

        rw::common::Ptr<rw::common::Log> _output;
        rw::common::PropertyMap _pmap;

        bool _isRunning;

        QTabWidget *_tabPane;
        rws::RobWorkStudio* _rws;
    };

}

#endif /* WORKCELLEDITORWINDOW_HPP_ */
