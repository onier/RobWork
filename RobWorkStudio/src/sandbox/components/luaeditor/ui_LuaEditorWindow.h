/********************************************************************************
** Form generated from reading ui file 'LuaEditorWindow.ui'
**
** Created: Thu 6. May 14:28:40 2010
**      by: Qt User Interface Compiler version 4.5.3
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

#ifndef UI_LUAEDITORWINDOW_H
#define UI_LUAEDITORWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LuaEditorWindow
{
public:
    QAction *actionNew;
    QAction *actionOpen;
    QAction *actionExit;
    QAction *actionAbout;
    QAction *actionToggle_Breakpoint;
    QAction *actionRemove_all_breakpoints;
    QAction *actionRestart;
    QAction *actionRestart_At_Line;
    QAction *actionResume;
    QAction *actionSuspend;
    QAction *actionTerminate;
    QAction *actionStep;
    QAction *actionRun_to_line;
    QAction *actionRun;
    QAction *actionDebug;
    QAction *actionTouch_up;
    QAction *actionClose;
    QAction *actionSave;
    QAction *actionSave_As;
    QWidget *centralwidget;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuEdit;
    QMenu *menuHelp;
    QMenu *menuRun;
    QMenu *menuTools;
    QToolBar *_fileToolBar;
    QToolBar *toolBar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *LuaEditorWindow)
    {
        if (LuaEditorWindow->objectName().isEmpty())
            LuaEditorWindow->setObjectName(QString::fromUtf8("LuaEditorWindow"));
        LuaEditorWindow->resize(517, 373);
        actionNew = new QAction(LuaEditorWindow);
        actionNew->setObjectName(QString::fromUtf8("actionNew"));
        actionOpen = new QAction(LuaEditorWindow);
        actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
        actionExit = new QAction(LuaEditorWindow);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        actionAbout = new QAction(LuaEditorWindow);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionToggle_Breakpoint = new QAction(LuaEditorWindow);
        actionToggle_Breakpoint->setObjectName(QString::fromUtf8("actionToggle_Breakpoint"));
        actionRemove_all_breakpoints = new QAction(LuaEditorWindow);
        actionRemove_all_breakpoints->setObjectName(QString::fromUtf8("actionRemove_all_breakpoints"));
        actionRestart = new QAction(LuaEditorWindow);
        actionRestart->setObjectName(QString::fromUtf8("actionRestart"));
        actionRestart_At_Line = new QAction(LuaEditorWindow);
        actionRestart_At_Line->setObjectName(QString::fromUtf8("actionRestart_At_Line"));
        actionResume = new QAction(LuaEditorWindow);
        actionResume->setObjectName(QString::fromUtf8("actionResume"));
        actionSuspend = new QAction(LuaEditorWindow);
        actionSuspend->setObjectName(QString::fromUtf8("actionSuspend"));
        actionTerminate = new QAction(LuaEditorWindow);
        actionTerminate->setObjectName(QString::fromUtf8("actionTerminate"));
        actionStep = new QAction(LuaEditorWindow);
        actionStep->setObjectName(QString::fromUtf8("actionStep"));
        actionRun_to_line = new QAction(LuaEditorWindow);
        actionRun_to_line->setObjectName(QString::fromUtf8("actionRun_to_line"));
        actionRun = new QAction(LuaEditorWindow);
        actionRun->setObjectName(QString::fromUtf8("actionRun"));
        actionDebug = new QAction(LuaEditorWindow);
        actionDebug->setObjectName(QString::fromUtf8("actionDebug"));
        actionTouch_up = new QAction(LuaEditorWindow);
        actionTouch_up->setObjectName(QString::fromUtf8("actionTouch_up"));
        actionClose = new QAction(LuaEditorWindow);
        actionClose->setObjectName(QString::fromUtf8("actionClose"));
        actionSave = new QAction(LuaEditorWindow);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        actionSave_As = new QAction(LuaEditorWindow);
        actionSave_As->setObjectName(QString::fromUtf8("actionSave_As"));
        centralwidget = new QWidget(LuaEditorWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        LuaEditorWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(LuaEditorWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 517, 21));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuEdit = new QMenu(menubar);
        menuEdit->setObjectName(QString::fromUtf8("menuEdit"));
        menuHelp = new QMenu(menubar);
        menuHelp->setObjectName(QString::fromUtf8("menuHelp"));
        menuRun = new QMenu(menubar);
        menuRun->setObjectName(QString::fromUtf8("menuRun"));
        menuTools = new QMenu(menubar);
        menuTools->setObjectName(QString::fromUtf8("menuTools"));
        LuaEditorWindow->setMenuBar(menubar);
        _fileToolBar = new QToolBar(LuaEditorWindow);
        _fileToolBar->setObjectName(QString::fromUtf8("_fileToolBar"));
        LuaEditorWindow->addToolBar(Qt::TopToolBarArea, _fileToolBar);
        toolBar = new QToolBar(LuaEditorWindow);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        LuaEditorWindow->addToolBar(Qt::TopToolBarArea, toolBar);
        statusbar = new QStatusBar(LuaEditorWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        LuaEditorWindow->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuEdit->menuAction());
        menubar->addAction(menuTools->menuAction());
        menubar->addAction(menuRun->menuAction());
        menubar->addAction(menuHelp->menuAction());
        menuFile->addAction(actionNew);
        menuFile->addAction(actionOpen);
        menuFile->addSeparator();
        menuFile->addAction(actionClose);
        menuFile->addSeparator();
        menuFile->addAction(actionSave);
        menuFile->addAction(actionSave_As);
        menuFile->addSeparator();
        menuFile->addAction(actionExit);
        menuHelp->addAction(actionAbout);
        menuRun->addAction(actionToggle_Breakpoint);
        menuRun->addAction(actionRemove_all_breakpoints);
        menuRun->addSeparator();
        menuRun->addAction(actionRestart_At_Line);
        menuRun->addAction(actionResume);
        menuRun->addAction(actionSuspend);
        menuRun->addAction(actionTerminate);
        menuRun->addAction(actionStep);
        menuRun->addAction(actionRun_to_line);
        menuRun->addSeparator();
        menuRun->addAction(actionRun);
        menuRun->addAction(actionDebug);
        menuTools->addAction(actionTouch_up);
        _fileToolBar->addSeparator();

        retranslateUi(LuaEditorWindow);

        QMetaObject::connectSlotsByName(LuaEditorWindow);
    } // setupUi

    void retranslateUi(QMainWindow *LuaEditorWindow)
    {
        LuaEditorWindow->setWindowTitle(QApplication::translate("LuaEditorWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        actionNew->setText(QApplication::translate("LuaEditorWindow", "New", 0, QApplication::UnicodeUTF8));
        actionNew->setShortcut(QApplication::translate("LuaEditorWindow", "Ctrl+Shift+N", 0, QApplication::UnicodeUTF8));
        actionOpen->setText(QApplication::translate("LuaEditorWindow", "Open", 0, QApplication::UnicodeUTF8));
        actionExit->setText(QApplication::translate("LuaEditorWindow", "Exit", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("LuaEditorWindow", "About", 0, QApplication::UnicodeUTF8));
        actionToggle_Breakpoint->setText(QApplication::translate("LuaEditorWindow", "Toggle Breakpoint", 0, QApplication::UnicodeUTF8));
        actionToggle_Breakpoint->setShortcut(QApplication::translate("LuaEditorWindow", "Ctrl+Shift+B", 0, QApplication::UnicodeUTF8));
        actionRemove_all_breakpoints->setText(QApplication::translate("LuaEditorWindow", "Remove all breakpoints", 0, QApplication::UnicodeUTF8));
        actionRestart->setText(QApplication::translate("LuaEditorWindow", "Restart", 0, QApplication::UnicodeUTF8));
        actionRestart_At_Line->setText(QApplication::translate("LuaEditorWindow", "Restart At Line", 0, QApplication::UnicodeUTF8));
        actionResume->setText(QApplication::translate("LuaEditorWindow", "Resume", 0, QApplication::UnicodeUTF8));
        actionSuspend->setText(QApplication::translate("LuaEditorWindow", "Suspend", 0, QApplication::UnicodeUTF8));
        actionTerminate->setText(QApplication::translate("LuaEditorWindow", "Terminate", 0, QApplication::UnicodeUTF8));
        actionStep->setText(QApplication::translate("LuaEditorWindow", "Step", 0, QApplication::UnicodeUTF8));
        actionStep->setShortcut(QApplication::translate("LuaEditorWindow", "Ctrl+Return", 0, QApplication::UnicodeUTF8));
        actionRun_to_line->setText(QApplication::translate("LuaEditorWindow", "Run to line", 0, QApplication::UnicodeUTF8));
        actionRun->setText(QApplication::translate("LuaEditorWindow", "Run", 0, QApplication::UnicodeUTF8));
        actionRun->setShortcut(QApplication::translate("LuaEditorWindow", "Ctrl+F11", "Ctrl+F11", QApplication::UnicodeUTF8));
        actionDebug->setText(QApplication::translate("LuaEditorWindow", "Debug", 0, QApplication::UnicodeUTF8));
        actionDebug->setShortcut(QApplication::translate("LuaEditorWindow", "F11", 0, QApplication::UnicodeUTF8));
        actionTouch_up->setText(QApplication::translate("LuaEditorWindow", "Touch-up state", 0, QApplication::UnicodeUTF8));
        actionTouch_up->setShortcut(QApplication::translate("LuaEditorWindow", "Ctrl+T", 0, QApplication::UnicodeUTF8));
        actionClose->setText(QApplication::translate("LuaEditorWindow", "Close", 0, QApplication::UnicodeUTF8));
        actionClose->setShortcut(QApplication::translate("LuaEditorWindow", "Ctrl+W", 0, QApplication::UnicodeUTF8));
        actionSave->setText(QApplication::translate("LuaEditorWindow", "Save", 0, QApplication::UnicodeUTF8));
        actionSave->setShortcut(QApplication::translate("LuaEditorWindow", "Ctrl+S", 0, QApplication::UnicodeUTF8));
        actionSave_As->setText(QApplication::translate("LuaEditorWindow", "Save As...", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("LuaEditorWindow", "File", 0, QApplication::UnicodeUTF8));
        menuEdit->setTitle(QApplication::translate("LuaEditorWindow", "Edit", 0, QApplication::UnicodeUTF8));
        menuHelp->setTitle(QApplication::translate("LuaEditorWindow", "Help", 0, QApplication::UnicodeUTF8));
        menuRun->setTitle(QApplication::translate("LuaEditorWindow", "Run", 0, QApplication::UnicodeUTF8));
        menuTools->setTitle(QApplication::translate("LuaEditorWindow", "Tools", 0, QApplication::UnicodeUTF8));
        _fileToolBar->setWindowTitle(QApplication::translate("LuaEditorWindow", "toolBar", 0, QApplication::UnicodeUTF8));
        toolBar->setWindowTitle(QApplication::translate("LuaEditorWindow", "toolBar", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class LuaEditorWindow: public Ui_LuaEditorWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LUAEDITORWINDOW_H
