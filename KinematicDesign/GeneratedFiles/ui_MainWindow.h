/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowClass
{
public:
    QAction *actionExit;
    QAction *actionSelect;
    QAction *actionFixedRectangle;
    QAction *actionFixedCircle;
    QAction *actionFixedPolygon;
    QAction *actionNew;
    QAction *actionCopy;
    QAction *actionSelectAll;
    QAction *actionDelete;
    QAction *actionPaste;
    QAction *actionUndo;
    QAction *actionRedo;
    QAction *actionOpen;
    QAction *actionSave;
    QAction *actionAddLayer;
    QAction *actionInsertLayer;
    QAction *actionDeleteLayer;
    QAction *actionCalculateSolution4RLinkage;
    QAction *actionCollisionCheck;
    QAction *actionLinkageRegion;
    QAction *actionKinematics;
    QAction *actionRun;
    QAction *actionRunBackward;
    QAction *actionStop;
    QAction *actionStepForward;
    QAction *actionStepBackward;
    QAction *actionCalculateSolutionSliderCrank;
    QAction *actionShowSolutions;
    QAction *actionRenderBasic;
    QAction *actionRenderSSAO;
    QAction *actionRenderLine;
    QAction *actionRenderHatching;
    QAction *actionSaveSTL;
    QAction *actionExportSTL;
    QAction *actionExportSCAD;
    QAction *actionOptions;
    QAction *actionMovingRectangle;
    QAction *actionMovingCircle;
    QAction *actionMovingPolygon;
    QAction *actionFixedRectangle_2;
    QAction *actionAho;
    QAction *actionLinkageAvoidanceRegion;
    QAction *actionCalculateSolutionWattI;
    QWidget *centralWidget;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuExport;
    QMenu *menuMode;
    QMenu *menuFixed_Body;
    QMenu *menuMovingBody;
    QMenu *menuEdit;
    QMenu *menuLayer;
    QMenu *menuKinematics;
    QMenu *menuView;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindowClass)
    {
        if (MainWindowClass->objectName().isEmpty())
            MainWindowClass->setObjectName(QStringLiteral("MainWindowClass"));
        MainWindowClass->resize(800, 853);
        actionExit = new QAction(MainWindowClass);
        actionExit->setObjectName(QStringLiteral("actionExit"));
        actionSelect = new QAction(MainWindowClass);
        actionSelect->setObjectName(QStringLiteral("actionSelect"));
        actionSelect->setCheckable(true);
        actionFixedRectangle = new QAction(MainWindowClass);
        actionFixedRectangle->setObjectName(QStringLiteral("actionFixedRectangle"));
        actionFixedRectangle->setCheckable(true);
        actionFixedCircle = new QAction(MainWindowClass);
        actionFixedCircle->setObjectName(QStringLiteral("actionFixedCircle"));
        actionFixedCircle->setCheckable(true);
        actionFixedPolygon = new QAction(MainWindowClass);
        actionFixedPolygon->setObjectName(QStringLiteral("actionFixedPolygon"));
        actionFixedPolygon->setCheckable(true);
        actionNew = new QAction(MainWindowClass);
        actionNew->setObjectName(QStringLiteral("actionNew"));
        actionCopy = new QAction(MainWindowClass);
        actionCopy->setObjectName(QStringLiteral("actionCopy"));
        actionSelectAll = new QAction(MainWindowClass);
        actionSelectAll->setObjectName(QStringLiteral("actionSelectAll"));
        actionDelete = new QAction(MainWindowClass);
        actionDelete->setObjectName(QStringLiteral("actionDelete"));
        actionPaste = new QAction(MainWindowClass);
        actionPaste->setObjectName(QStringLiteral("actionPaste"));
        actionUndo = new QAction(MainWindowClass);
        actionUndo->setObjectName(QStringLiteral("actionUndo"));
        actionRedo = new QAction(MainWindowClass);
        actionRedo->setObjectName(QStringLiteral("actionRedo"));
        actionOpen = new QAction(MainWindowClass);
        actionOpen->setObjectName(QStringLiteral("actionOpen"));
        actionSave = new QAction(MainWindowClass);
        actionSave->setObjectName(QStringLiteral("actionSave"));
        actionAddLayer = new QAction(MainWindowClass);
        actionAddLayer->setObjectName(QStringLiteral("actionAddLayer"));
        actionInsertLayer = new QAction(MainWindowClass);
        actionInsertLayer->setObjectName(QStringLiteral("actionInsertLayer"));
        actionDeleteLayer = new QAction(MainWindowClass);
        actionDeleteLayer->setObjectName(QStringLiteral("actionDeleteLayer"));
        actionCalculateSolution4RLinkage = new QAction(MainWindowClass);
        actionCalculateSolution4RLinkage->setObjectName(QStringLiteral("actionCalculateSolution4RLinkage"));
        actionCollisionCheck = new QAction(MainWindowClass);
        actionCollisionCheck->setObjectName(QStringLiteral("actionCollisionCheck"));
        actionCollisionCheck->setCheckable(true);
        actionLinkageRegion = new QAction(MainWindowClass);
        actionLinkageRegion->setObjectName(QStringLiteral("actionLinkageRegion"));
        actionLinkageRegion->setCheckable(true);
        actionKinematics = new QAction(MainWindowClass);
        actionKinematics->setObjectName(QStringLiteral("actionKinematics"));
        actionKinematics->setCheckable(true);
        actionRun = new QAction(MainWindowClass);
        actionRun->setObjectName(QStringLiteral("actionRun"));
        actionRunBackward = new QAction(MainWindowClass);
        actionRunBackward->setObjectName(QStringLiteral("actionRunBackward"));
        actionStop = new QAction(MainWindowClass);
        actionStop->setObjectName(QStringLiteral("actionStop"));
        actionStepForward = new QAction(MainWindowClass);
        actionStepForward->setObjectName(QStringLiteral("actionStepForward"));
        actionStepBackward = new QAction(MainWindowClass);
        actionStepBackward->setObjectName(QStringLiteral("actionStepBackward"));
        actionCalculateSolutionSliderCrank = new QAction(MainWindowClass);
        actionCalculateSolutionSliderCrank->setObjectName(QStringLiteral("actionCalculateSolutionSliderCrank"));
        actionShowSolutions = new QAction(MainWindowClass);
        actionShowSolutions->setObjectName(QStringLiteral("actionShowSolutions"));
        actionShowSolutions->setCheckable(true);
        actionRenderBasic = new QAction(MainWindowClass);
        actionRenderBasic->setObjectName(QStringLiteral("actionRenderBasic"));
        actionRenderBasic->setCheckable(true);
        actionRenderSSAO = new QAction(MainWindowClass);
        actionRenderSSAO->setObjectName(QStringLiteral("actionRenderSSAO"));
        actionRenderSSAO->setCheckable(true);
        actionRenderLine = new QAction(MainWindowClass);
        actionRenderLine->setObjectName(QStringLiteral("actionRenderLine"));
        actionRenderLine->setCheckable(true);
        actionRenderHatching = new QAction(MainWindowClass);
        actionRenderHatching->setObjectName(QStringLiteral("actionRenderHatching"));
        actionRenderHatching->setCheckable(true);
        actionSaveSTL = new QAction(MainWindowClass);
        actionSaveSTL->setObjectName(QStringLiteral("actionSaveSTL"));
        actionExportSTL = new QAction(MainWindowClass);
        actionExportSTL->setObjectName(QStringLiteral("actionExportSTL"));
        actionExportSCAD = new QAction(MainWindowClass);
        actionExportSCAD->setObjectName(QStringLiteral("actionExportSCAD"));
        actionOptions = new QAction(MainWindowClass);
        actionOptions->setObjectName(QStringLiteral("actionOptions"));
        actionMovingRectangle = new QAction(MainWindowClass);
        actionMovingRectangle->setObjectName(QStringLiteral("actionMovingRectangle"));
        actionMovingRectangle->setCheckable(true);
        actionMovingCircle = new QAction(MainWindowClass);
        actionMovingCircle->setObjectName(QStringLiteral("actionMovingCircle"));
        actionMovingCircle->setCheckable(true);
        actionMovingPolygon = new QAction(MainWindowClass);
        actionMovingPolygon->setObjectName(QStringLiteral("actionMovingPolygon"));
        actionMovingPolygon->setCheckable(true);
        actionFixedRectangle_2 = new QAction(MainWindowClass);
        actionFixedRectangle_2->setObjectName(QStringLiteral("actionFixedRectangle_2"));
        actionAho = new QAction(MainWindowClass);
        actionAho->setObjectName(QStringLiteral("actionAho"));
        actionLinkageAvoidanceRegion = new QAction(MainWindowClass);
        actionLinkageAvoidanceRegion->setObjectName(QStringLiteral("actionLinkageAvoidanceRegion"));
        actionLinkageAvoidanceRegion->setCheckable(true);
        actionCalculateSolutionWattI = new QAction(MainWindowClass);
        actionCalculateSolutionWattI->setObjectName(QStringLiteral("actionCalculateSolutionWattI"));
        centralWidget = new QWidget(MainWindowClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        MainWindowClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindowClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 800, 21));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        menuExport = new QMenu(menuFile);
        menuExport->setObjectName(QStringLiteral("menuExport"));
        menuMode = new QMenu(menuBar);
        menuMode->setObjectName(QStringLiteral("menuMode"));
        menuFixed_Body = new QMenu(menuMode);
        menuFixed_Body->setObjectName(QStringLiteral("menuFixed_Body"));
        menuMovingBody = new QMenu(menuMode);
        menuMovingBody->setObjectName(QStringLiteral("menuMovingBody"));
        menuEdit = new QMenu(menuBar);
        menuEdit->setObjectName(QStringLiteral("menuEdit"));
        menuLayer = new QMenu(menuBar);
        menuLayer->setObjectName(QStringLiteral("menuLayer"));
        menuKinematics = new QMenu(menuBar);
        menuKinematics->setObjectName(QStringLiteral("menuKinematics"));
        menuView = new QMenu(menuBar);
        menuView->setObjectName(QStringLiteral("menuView"));
        MainWindowClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindowClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindowClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindowClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindowClass->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuEdit->menuAction());
        menuBar->addAction(menuMode->menuAction());
        menuBar->addAction(menuLayer->menuAction());
        menuBar->addAction(menuKinematics->menuAction());
        menuBar->addAction(menuView->menuAction());
        menuFile->addAction(actionNew);
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionSave);
        menuFile->addAction(menuExport->menuAction());
        menuFile->addSeparator();
        menuFile->addAction(actionExit);
        menuExport->addAction(actionExportSTL);
        menuExport->addAction(actionExportSCAD);
        menuMode->addAction(actionSelect);
        menuMode->addSeparator();
        menuMode->addAction(menuFixed_Body->menuAction());
        menuMode->addAction(menuMovingBody->menuAction());
        menuMode->addSeparator();
        menuMode->addAction(actionLinkageRegion);
        menuMode->addAction(actionLinkageAvoidanceRegion);
        menuMode->addSeparator();
        menuMode->addAction(actionKinematics);
        menuFixed_Body->addAction(actionFixedRectangle);
        menuFixed_Body->addAction(actionFixedCircle);
        menuFixed_Body->addAction(actionFixedPolygon);
        menuMovingBody->addAction(actionMovingRectangle);
        menuMovingBody->addAction(actionMovingCircle);
        menuMovingBody->addAction(actionMovingPolygon);
        menuEdit->addAction(actionUndo);
        menuEdit->addAction(actionRedo);
        menuEdit->addSeparator();
        menuEdit->addAction(actionCopy);
        menuEdit->addAction(actionPaste);
        menuEdit->addSeparator();
        menuEdit->addAction(actionDelete);
        menuEdit->addAction(actionSelectAll);
        menuLayer->addAction(actionAddLayer);
        menuLayer->addAction(actionInsertLayer);
        menuLayer->addAction(actionDeleteLayer);
        menuLayer->addSeparator();
        menuKinematics->addAction(actionCalculateSolution4RLinkage);
        menuKinematics->addAction(actionCalculateSolutionSliderCrank);
        menuKinematics->addAction(actionCalculateSolutionWattI);
        menuKinematics->addSeparator();
        menuKinematics->addAction(actionRun);
        menuKinematics->addAction(actionRunBackward);
        menuKinematics->addAction(actionStop);
        menuKinematics->addSeparator();
        menuKinematics->addAction(actionStepForward);
        menuKinematics->addAction(actionStepBackward);
        menuKinematics->addSeparator();
        menuKinematics->addAction(actionCollisionCheck);
        menuKinematics->addAction(actionOptions);
        menuView->addAction(actionShowSolutions);
        menuView->addSeparator();
        menuView->addAction(actionRenderBasic);
        menuView->addAction(actionRenderSSAO);
        menuView->addAction(actionRenderLine);
        menuView->addAction(actionRenderHatching);

        retranslateUi(MainWindowClass);

        QMetaObject::connectSlotsByName(MainWindowClass);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowClass)
    {
        MainWindowClass->setWindowTitle(QApplication::translate("MainWindowClass", "Canvas 3D", 0));
        actionExit->setText(QApplication::translate("MainWindowClass", "Exit", 0));
        actionSelect->setText(QApplication::translate("MainWindowClass", "Select", 0));
        actionFixedRectangle->setText(QApplication::translate("MainWindowClass", "Rectangle", 0));
        actionFixedCircle->setText(QApplication::translate("MainWindowClass", "Circle", 0));
        actionFixedPolygon->setText(QApplication::translate("MainWindowClass", "Polygon", 0));
        actionNew->setText(QApplication::translate("MainWindowClass", "New", 0));
        actionNew->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+N", 0));
        actionCopy->setText(QApplication::translate("MainWindowClass", "Copy", 0));
        actionCopy->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+C", 0));
        actionSelectAll->setText(QApplication::translate("MainWindowClass", "Select All", 0));
        actionSelectAll->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+A", 0));
        actionDelete->setText(QApplication::translate("MainWindowClass", "Delete", 0));
        actionDelete->setShortcut(QApplication::translate("MainWindowClass", "Del", 0));
        actionPaste->setText(QApplication::translate("MainWindowClass", "Paste", 0));
        actionPaste->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+V", 0));
        actionUndo->setText(QApplication::translate("MainWindowClass", "Undo", 0));
        actionUndo->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+Z", 0));
        actionRedo->setText(QApplication::translate("MainWindowClass", "Redo", 0));
        actionRedo->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+Y", 0));
        actionOpen->setText(QApplication::translate("MainWindowClass", "Open", 0));
        actionOpen->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+O", 0));
        actionSave->setText(QApplication::translate("MainWindowClass", "Save", 0));
        actionSave->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+S", 0));
        actionAddLayer->setText(QApplication::translate("MainWindowClass", "Add Layer", 0));
        actionInsertLayer->setText(QApplication::translate("MainWindowClass", "Insert Layer", 0));
        actionDeleteLayer->setText(QApplication::translate("MainWindowClass", "Delete Layer", 0));
        actionCalculateSolution4RLinkage->setText(QApplication::translate("MainWindowClass", "Calculate Solution for 4R Linkage", 0));
        actionCollisionCheck->setText(QApplication::translate("MainWindowClass", "Collision Check", 0));
        actionLinkageRegion->setText(QApplication::translate("MainWindowClass", "Linkage Region", 0));
        actionKinematics->setText(QApplication::translate("MainWindowClass", "Kinematics", 0));
        actionRun->setText(QApplication::translate("MainWindowClass", "Run", 0));
        actionRunBackward->setText(QApplication::translate("MainWindowClass", "Run Backward", 0));
        actionStop->setText(QApplication::translate("MainWindowClass", "Stop", 0));
        actionStepForward->setText(QApplication::translate("MainWindowClass", "Step Forward", 0));
        actionStepForward->setShortcut(QApplication::translate("MainWindowClass", "Right", 0));
        actionStepBackward->setText(QApplication::translate("MainWindowClass", "Step Backward", 0));
        actionStepBackward->setShortcut(QApplication::translate("MainWindowClass", "Left", 0));
        actionCalculateSolutionSliderCrank->setText(QApplication::translate("MainWindowClass", "Calculate Solution for Slider Crank", 0));
        actionShowSolutions->setText(QApplication::translate("MainWindowClass", "Show Solutions", 0));
        actionRenderBasic->setText(QApplication::translate("MainWindowClass", "Basic", 0));
        actionRenderSSAO->setText(QApplication::translate("MainWindowClass", "SSAO", 0));
        actionRenderLine->setText(QApplication::translate("MainWindowClass", "Line", 0));
        actionRenderHatching->setText(QApplication::translate("MainWindowClass", "Hatching", 0));
        actionSaveSTL->setText(QApplication::translate("MainWindowClass", "Save STL", 0));
        actionExportSTL->setText(QApplication::translate("MainWindowClass", "STL", 0));
        actionExportSCAD->setText(QApplication::translate("MainWindowClass", "SCAD", 0));
        actionOptions->setText(QApplication::translate("MainWindowClass", "Options", 0));
        actionMovingRectangle->setText(QApplication::translate("MainWindowClass", "Rectangle", 0));
        actionMovingCircle->setText(QApplication::translate("MainWindowClass", "Circle", 0));
        actionMovingPolygon->setText(QApplication::translate("MainWindowClass", "Polygon", 0));
        actionFixedRectangle_2->setText(QApplication::translate("MainWindowClass", "Rectangle", 0));
        actionAho->setText(QApplication::translate("MainWindowClass", "Aho", 0));
        actionLinkageAvoidanceRegion->setText(QApplication::translate("MainWindowClass", "Linkage Avoidance Region", 0));
        actionCalculateSolutionWattI->setText(QApplication::translate("MainWindowClass", "Calculate Solution for Watt I", 0));
        menuFile->setTitle(QApplication::translate("MainWindowClass", "File", 0));
        menuExport->setTitle(QApplication::translate("MainWindowClass", "Export", 0));
        menuMode->setTitle(QApplication::translate("MainWindowClass", "Mode", 0));
        menuFixed_Body->setTitle(QApplication::translate("MainWindowClass", "Fixed Body", 0));
        menuMovingBody->setTitle(QApplication::translate("MainWindowClass", "Moving Body", 0));
        menuEdit->setTitle(QApplication::translate("MainWindowClass", "Edit", 0));
        menuLayer->setTitle(QApplication::translate("MainWindowClass", "Layer", 0));
        menuKinematics->setTitle(QApplication::translate("MainWindowClass", "Kinematics", 0));
        menuView->setTitle(QApplication::translate("MainWindowClass", "View", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindowClass: public Ui_MainWindowClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
