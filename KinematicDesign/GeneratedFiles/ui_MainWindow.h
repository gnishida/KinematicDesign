/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
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
    QAction *actionGenerateLinkage;
    QAction *actionCollisionCheck;
    QAction *actionLinkageRegion;
    QAction *actionKinematics;
    QAction *actionRun;
    QAction *actionRunBackward;
    QAction *actionStop;
    QAction *actionStepForward;
    QAction *actionStepBackward;
    QAction *actionGenerateSliderCrank;
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
    QAction *actionLinkageAvoidance;
    QAction *actionGenerateWattI;
    QAction *actionShowGridLines;
    QAction *actionShowInputPoses;
    QAction *actionSaveImage;
    QAction *actionRestrictMotionRange;
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
        QIcon icon;
        icon.addFile(QStringLiteral("Resources/select.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelect->setIcon(icon);
        actionFixedRectangle = new QAction(MainWindowClass);
        actionFixedRectangle->setObjectName(QStringLiteral("actionFixedRectangle"));
        actionFixedRectangle->setCheckable(true);
        QIcon icon1;
        icon1.addFile(QStringLiteral("Resources/fixed_rectangle.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionFixedRectangle->setIcon(icon1);
        actionFixedCircle = new QAction(MainWindowClass);
        actionFixedCircle->setObjectName(QStringLiteral("actionFixedCircle"));
        actionFixedCircle->setCheckable(true);
        QIcon icon2;
        icon2.addFile(QStringLiteral("Resources/fixed_circle.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionFixedCircle->setIcon(icon2);
        actionFixedPolygon = new QAction(MainWindowClass);
        actionFixedPolygon->setObjectName(QStringLiteral("actionFixedPolygon"));
        actionFixedPolygon->setCheckable(true);
        QIcon icon3;
        icon3.addFile(QStringLiteral("Resources/fixed_polygon.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionFixedPolygon->setIcon(icon3);
        actionNew = new QAction(MainWindowClass);
        actionNew->setObjectName(QStringLiteral("actionNew"));
        QIcon icon4;
        icon4.addFile(QStringLiteral("Resources/new.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionNew->setIcon(icon4);
        actionCopy = new QAction(MainWindowClass);
        actionCopy->setObjectName(QStringLiteral("actionCopy"));
        QIcon icon5;
        icon5.addFile(QStringLiteral("Resources/copy.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionCopy->setIcon(icon5);
        actionSelectAll = new QAction(MainWindowClass);
        actionSelectAll->setObjectName(QStringLiteral("actionSelectAll"));
        actionDelete = new QAction(MainWindowClass);
        actionDelete->setObjectName(QStringLiteral("actionDelete"));
        QIcon icon6;
        icon6.addFile(QStringLiteral("Resources/delete.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionDelete->setIcon(icon6);
        actionPaste = new QAction(MainWindowClass);
        actionPaste->setObjectName(QStringLiteral("actionPaste"));
        QIcon icon7;
        icon7.addFile(QStringLiteral("Resources/paste.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionPaste->setIcon(icon7);
        actionUndo = new QAction(MainWindowClass);
        actionUndo->setObjectName(QStringLiteral("actionUndo"));
        QIcon icon8;
        icon8.addFile(QStringLiteral("Resources/undo.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionUndo->setIcon(icon8);
        actionRedo = new QAction(MainWindowClass);
        actionRedo->setObjectName(QStringLiteral("actionRedo"));
        QIcon icon9;
        icon9.addFile(QStringLiteral("Resources/redo.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionRedo->setIcon(icon9);
        actionOpen = new QAction(MainWindowClass);
        actionOpen->setObjectName(QStringLiteral("actionOpen"));
        QIcon icon10;
        icon10.addFile(QStringLiteral("Resources/open.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionOpen->setIcon(icon10);
        actionSave = new QAction(MainWindowClass);
        actionSave->setObjectName(QStringLiteral("actionSave"));
        QIcon icon11;
        icon11.addFile(QStringLiteral("Resources/save.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSave->setIcon(icon11);
        actionAddLayer = new QAction(MainWindowClass);
        actionAddLayer->setObjectName(QStringLiteral("actionAddLayer"));
        actionInsertLayer = new QAction(MainWindowClass);
        actionInsertLayer->setObjectName(QStringLiteral("actionInsertLayer"));
        actionDeleteLayer = new QAction(MainWindowClass);
        actionDeleteLayer->setObjectName(QStringLiteral("actionDeleteLayer"));
        actionGenerateLinkage = new QAction(MainWindowClass);
        actionGenerateLinkage->setObjectName(QStringLiteral("actionGenerateLinkage"));
        QIcon icon12;
        icon12.addFile(QStringLiteral("Resources/fourbar_linkage.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionGenerateLinkage->setIcon(icon12);
        actionCollisionCheck = new QAction(MainWindowClass);
        actionCollisionCheck->setObjectName(QStringLiteral("actionCollisionCheck"));
        actionCollisionCheck->setCheckable(true);
        actionLinkageRegion = new QAction(MainWindowClass);
        actionLinkageRegion->setObjectName(QStringLiteral("actionLinkageRegion"));
        actionLinkageRegion->setCheckable(true);
        QIcon icon13;
        icon13.addFile(QStringLiteral("Resources/linkage_region.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionLinkageRegion->setIcon(icon13);
        actionKinematics = new QAction(MainWindowClass);
        actionKinematics->setObjectName(QStringLiteral("actionKinematics"));
        actionKinematics->setCheckable(true);
        actionRun = new QAction(MainWindowClass);
        actionRun->setObjectName(QStringLiteral("actionRun"));
        QIcon icon14;
        icon14.addFile(QStringLiteral("Resources/run.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionRun->setIcon(icon14);
        actionRunBackward = new QAction(MainWindowClass);
        actionRunBackward->setObjectName(QStringLiteral("actionRunBackward"));
        actionStop = new QAction(MainWindowClass);
        actionStop->setObjectName(QStringLiteral("actionStop"));
        QIcon icon15;
        icon15.addFile(QStringLiteral("Resources/stop.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionStop->setIcon(icon15);
        actionStepForward = new QAction(MainWindowClass);
        actionStepForward->setObjectName(QStringLiteral("actionStepForward"));
        QIcon icon16;
        icon16.addFile(QStringLiteral("Resources/step_forward.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionStepForward->setIcon(icon16);
        actionStepBackward = new QAction(MainWindowClass);
        actionStepBackward->setObjectName(QStringLiteral("actionStepBackward"));
        QIcon icon17;
        icon17.addFile(QStringLiteral("Resources/step_backward.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionStepBackward->setIcon(icon17);
        actionGenerateSliderCrank = new QAction(MainWindowClass);
        actionGenerateSliderCrank->setObjectName(QStringLiteral("actionGenerateSliderCrank"));
        QIcon icon18;
        icon18.addFile(QStringLiteral("Resources/slider_crank.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionGenerateSliderCrank->setIcon(icon18);
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
        QIcon icon19;
        icon19.addFile(QStringLiteral("Resources/options.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionOptions->setIcon(icon19);
        actionMovingRectangle = new QAction(MainWindowClass);
        actionMovingRectangle->setObjectName(QStringLiteral("actionMovingRectangle"));
        actionMovingRectangle->setCheckable(true);
        QIcon icon20;
        icon20.addFile(QStringLiteral("Resources/moving_rectangle.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionMovingRectangle->setIcon(icon20);
        actionMovingCircle = new QAction(MainWindowClass);
        actionMovingCircle->setObjectName(QStringLiteral("actionMovingCircle"));
        actionMovingCircle->setCheckable(true);
        QIcon icon21;
        icon21.addFile(QStringLiteral("Resources/moving_circle.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionMovingCircle->setIcon(icon21);
        actionMovingPolygon = new QAction(MainWindowClass);
        actionMovingPolygon->setObjectName(QStringLiteral("actionMovingPolygon"));
        actionMovingPolygon->setCheckable(true);
        QIcon icon22;
        icon22.addFile(QStringLiteral("Resources/moving_polygon.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionMovingPolygon->setIcon(icon22);
        actionFixedRectangle_2 = new QAction(MainWindowClass);
        actionFixedRectangle_2->setObjectName(QStringLiteral("actionFixedRectangle_2"));
        actionAho = new QAction(MainWindowClass);
        actionAho->setObjectName(QStringLiteral("actionAho"));
        actionLinkageAvoidance = new QAction(MainWindowClass);
        actionLinkageAvoidance->setObjectName(QStringLiteral("actionLinkageAvoidance"));
        actionLinkageAvoidance->setCheckable(true);
        QIcon icon23;
        icon23.addFile(QStringLiteral("Resources/linkage_avoidance.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionLinkageAvoidance->setIcon(icon23);
        actionGenerateWattI = new QAction(MainWindowClass);
        actionGenerateWattI->setObjectName(QStringLiteral("actionGenerateWattI"));
        QIcon icon24;
        icon24.addFile(QStringLiteral("Resources/watt_i_linkage.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionGenerateWattI->setIcon(icon24);
        actionShowGridLines = new QAction(MainWindowClass);
        actionShowGridLines->setObjectName(QStringLiteral("actionShowGridLines"));
        actionShowGridLines->setCheckable(true);
        actionShowGridLines->setChecked(true);
        actionShowInputPoses = new QAction(MainWindowClass);
        actionShowInputPoses->setObjectName(QStringLiteral("actionShowInputPoses"));
        actionShowInputPoses->setCheckable(true);
        actionShowInputPoses->setChecked(true);
        actionSaveImage = new QAction(MainWindowClass);
        actionSaveImage->setObjectName(QStringLiteral("actionSaveImage"));
        actionRestrictMotionRange = new QAction(MainWindowClass);
        actionRestrictMotionRange->setObjectName(QStringLiteral("actionRestrictMotionRange"));
        actionRestrictMotionRange->setCheckable(true);
        actionRestrictMotionRange->setChecked(true);
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
        mainToolBar->setMinimumSize(QSize(0, 0));
        MainWindowClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindowClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        statusBar->setMinimumSize(QSize(0, 0));
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
        menuFile->addSeparator();
        menuFile->addAction(actionSaveImage);
        menuFile->addSeparator();
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
        menuMode->addAction(actionLinkageAvoidance);
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
        menuKinematics->addAction(actionGenerateLinkage);
        menuKinematics->addSeparator();
        menuKinematics->addAction(actionRun);
        menuKinematics->addAction(actionRunBackward);
        menuKinematics->addAction(actionStop);
        menuKinematics->addSeparator();
        menuKinematics->addAction(actionStepForward);
        menuKinematics->addAction(actionStepBackward);
        menuKinematics->addSeparator();
        menuKinematics->addAction(actionCollisionCheck);
        menuKinematics->addAction(actionRestrictMotionRange);
        menuKinematics->addAction(actionOptions);
        menuView->addAction(actionShowSolutions);
        menuView->addAction(actionShowGridLines);
        menuView->addAction(actionShowInputPoses);
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
        MainWindowClass->setWindowTitle(QApplication::translate("MainWindowClass", "Kinematic Design", Q_NULLPTR));
        actionExit->setText(QApplication::translate("MainWindowClass", "Exit", Q_NULLPTR));
        actionSelect->setText(QApplication::translate("MainWindowClass", "Select", Q_NULLPTR));
        actionFixedRectangle->setText(QApplication::translate("MainWindowClass", "Rectangle", Q_NULLPTR));
        actionFixedCircle->setText(QApplication::translate("MainWindowClass", "Circle", Q_NULLPTR));
        actionFixedPolygon->setText(QApplication::translate("MainWindowClass", "Polygon", Q_NULLPTR));
        actionNew->setText(QApplication::translate("MainWindowClass", "New", Q_NULLPTR));
        actionNew->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+N", Q_NULLPTR));
        actionCopy->setText(QApplication::translate("MainWindowClass", "Copy", Q_NULLPTR));
        actionCopy->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+C", Q_NULLPTR));
        actionSelectAll->setText(QApplication::translate("MainWindowClass", "Select All", Q_NULLPTR));
        actionSelectAll->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+A", Q_NULLPTR));
        actionDelete->setText(QApplication::translate("MainWindowClass", "Delete", Q_NULLPTR));
        actionDelete->setShortcut(QApplication::translate("MainWindowClass", "Del", Q_NULLPTR));
        actionPaste->setText(QApplication::translate("MainWindowClass", "Paste", Q_NULLPTR));
        actionPaste->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+V", Q_NULLPTR));
        actionUndo->setText(QApplication::translate("MainWindowClass", "Undo", Q_NULLPTR));
        actionUndo->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+Z", Q_NULLPTR));
        actionRedo->setText(QApplication::translate("MainWindowClass", "Redo", Q_NULLPTR));
        actionRedo->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+Y", Q_NULLPTR));
        actionOpen->setText(QApplication::translate("MainWindowClass", "Open", Q_NULLPTR));
        actionOpen->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+O", Q_NULLPTR));
        actionSave->setText(QApplication::translate("MainWindowClass", "Save", Q_NULLPTR));
        actionSave->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+S", Q_NULLPTR));
        actionAddLayer->setText(QApplication::translate("MainWindowClass", "Add Layer", Q_NULLPTR));
        actionInsertLayer->setText(QApplication::translate("MainWindowClass", "Insert Layer", Q_NULLPTR));
        actionDeleteLayer->setText(QApplication::translate("MainWindowClass", "Delete Layer", Q_NULLPTR));
        actionGenerateLinkage->setText(QApplication::translate("MainWindowClass", "Generate Linkage", Q_NULLPTR));
        actionCollisionCheck->setText(QApplication::translate("MainWindowClass", "Collision Check", Q_NULLPTR));
        actionLinkageRegion->setText(QApplication::translate("MainWindowClass", "Linkage Region", Q_NULLPTR));
        actionKinematics->setText(QApplication::translate("MainWindowClass", "Kinematics", Q_NULLPTR));
        actionRun->setText(QApplication::translate("MainWindowClass", "Run", Q_NULLPTR));
        actionRunBackward->setText(QApplication::translate("MainWindowClass", "Run Backward", Q_NULLPTR));
        actionStop->setText(QApplication::translate("MainWindowClass", "Stop", Q_NULLPTR));
        actionStepForward->setText(QApplication::translate("MainWindowClass", "Step Forward", Q_NULLPTR));
        actionStepForward->setShortcut(QApplication::translate("MainWindowClass", "Right", Q_NULLPTR));
        actionStepBackward->setText(QApplication::translate("MainWindowClass", "Step Backward", Q_NULLPTR));
        actionStepBackward->setShortcut(QApplication::translate("MainWindowClass", "Left", Q_NULLPTR));
        actionGenerateSliderCrank->setText(QApplication::translate("MainWindowClass", "Generate Slider Crank", Q_NULLPTR));
        actionShowSolutions->setText(QApplication::translate("MainWindowClass", "Show Solutions", Q_NULLPTR));
        actionRenderBasic->setText(QApplication::translate("MainWindowClass", "Basic", Q_NULLPTR));
        actionRenderSSAO->setText(QApplication::translate("MainWindowClass", "SSAO", Q_NULLPTR));
        actionRenderLine->setText(QApplication::translate("MainWindowClass", "Line", Q_NULLPTR));
        actionRenderHatching->setText(QApplication::translate("MainWindowClass", "Hatching", Q_NULLPTR));
        actionSaveSTL->setText(QApplication::translate("MainWindowClass", "Save STL", Q_NULLPTR));
        actionExportSTL->setText(QApplication::translate("MainWindowClass", "STL", Q_NULLPTR));
        actionExportSCAD->setText(QApplication::translate("MainWindowClass", "SCAD", Q_NULLPTR));
        actionOptions->setText(QApplication::translate("MainWindowClass", "Options", Q_NULLPTR));
        actionMovingRectangle->setText(QApplication::translate("MainWindowClass", "Rectangle", Q_NULLPTR));
        actionMovingCircle->setText(QApplication::translate("MainWindowClass", "Circle", Q_NULLPTR));
        actionMovingPolygon->setText(QApplication::translate("MainWindowClass", "Polygon", Q_NULLPTR));
        actionFixedRectangle_2->setText(QApplication::translate("MainWindowClass", "Rectangle", Q_NULLPTR));
        actionAho->setText(QApplication::translate("MainWindowClass", "Aho", Q_NULLPTR));
        actionLinkageAvoidance->setText(QApplication::translate("MainWindowClass", "Linkage Avoidance Region", Q_NULLPTR));
        actionGenerateWattI->setText(QApplication::translate("MainWindowClass", "Generate Watt I", Q_NULLPTR));
        actionShowGridLines->setText(QApplication::translate("MainWindowClass", "Show Grid Lines", Q_NULLPTR));
        actionShowInputPoses->setText(QApplication::translate("MainWindowClass", "Show Input Poses", Q_NULLPTR));
        actionSaveImage->setText(QApplication::translate("MainWindowClass", "Save Image", Q_NULLPTR));
        actionSaveImage->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+P", Q_NULLPTR));
        actionRestrictMotionRange->setText(QApplication::translate("MainWindowClass", "Restrict Motion Range", Q_NULLPTR));
        menuFile->setTitle(QApplication::translate("MainWindowClass", "File", Q_NULLPTR));
        menuExport->setTitle(QApplication::translate("MainWindowClass", "Export", Q_NULLPTR));
        menuMode->setTitle(QApplication::translate("MainWindowClass", "Mode", Q_NULLPTR));
        menuFixed_Body->setTitle(QApplication::translate("MainWindowClass", "Fixed Body", Q_NULLPTR));
        menuMovingBody->setTitle(QApplication::translate("MainWindowClass", "Moving Body", Q_NULLPTR));
        menuEdit->setTitle(QApplication::translate("MainWindowClass", "Edit", Q_NULLPTR));
        menuLayer->setTitle(QApplication::translate("MainWindowClass", "Layer", Q_NULLPTR));
        menuKinematics->setTitle(QApplication::translate("MainWindowClass", "Kinematics", Q_NULLPTR));
        menuView->setTitle(QApplication::translate("MainWindowClass", "View", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindowClass: public Ui_MainWindowClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
