#include "MainWindow.h"
#include <QFileDialog>
#include "LinkageSynthesisOptionDialog.h"
#include "OptionDialog.h"
#include <QDateTime>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	ui.setupUi(this);

	glWidget = new GLWidget3D(this);
	setCentralWidget(glWidget);

	ui.actionCollisionCheck->setChecked(glWidget->collision_check);
	ui.actionShowSolutions->setChecked(glWidget->show_solutions);

	// group for mode
	QActionGroup* groupMode = new QActionGroup(this);
	groupMode->addAction(ui.actionSelect);
	groupMode->addAction(ui.actionFixedRectangle);
	groupMode->addAction(ui.actionFixedCircle);
	groupMode->addAction(ui.actionFixedPolygon);
	groupMode->addAction(ui.actionMovingRectangle);
	groupMode->addAction(ui.actionMovingCircle);
	groupMode->addAction(ui.actionMovingPolygon);
	groupMode->addAction(ui.actionLinkageRegion);
	groupMode->addAction(ui.actionLinkageAvoidance);
	groupMode->addAction(ui.actionKinematics);
	ui.actionSelect->setChecked(true);

	// group for layer
	groupLayer = new QActionGroup(this);
	initLayerMenu(2);

	// group for rendering
	QActionGroup* groupRendering = new QActionGroup(this);
	groupRendering->addAction(ui.actionRenderBasic);
	groupRendering->addAction(ui.actionRenderSSAO);
	groupRendering->addAction(ui.actionRenderLine);
	groupRendering->addAction(ui.actionRenderHatching);

	connect(ui.actionNew, SIGNAL(triggered()), this, SLOT(onNew()));
	connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onOpen()));
	connect(ui.actionSave, SIGNAL(triggered()), this, SLOT(onSave()));
	connect(ui.actionSaveImage, SIGNAL(triggered()), this, SLOT(onSaveImage()));
	connect(ui.actionExportSTL, SIGNAL(triggered()), this, SLOT(onExportSTL()));
	connect(ui.actionExportSCAD, SIGNAL(triggered()), this, SLOT(onExportSCAD()));
	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.actionUndo, SIGNAL(triggered()), this, SLOT(onUndo()));
	connect(ui.actionRedo, SIGNAL(triggered()), this, SLOT(onRedo()));
	connect(ui.actionCopy, SIGNAL(triggered()), this, SLOT(onCopy()));
	connect(ui.actionPaste, SIGNAL(triggered()), this, SLOT(onPaste()));
	connect(ui.actionDelete, SIGNAL(triggered()), this, SLOT(onDelete()));
	connect(ui.actionSelectAll, SIGNAL(triggered()), this, SLOT(onSelectAll()));
	connect(ui.actionSelect, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionFixedRectangle, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionFixedCircle, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionFixedPolygon, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionMovingRectangle, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionMovingCircle, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionMovingPolygon, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionLinkageRegion, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionLinkageAvoidance, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionKinematics, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionAddLayer, SIGNAL(triggered()), this, SLOT(onAddLayer()));
	connect(ui.actionInsertLayer, SIGNAL(triggered()), this, SLOT(onInsertLayer()));
	connect(ui.actionDeleteLayer, SIGNAL(triggered()), this, SLOT(onDeleteLayer()));
	connect(ui.actionGenerateLinkage, SIGNAL(triggered()), this, SLOT(onGenerateLinkage()));
	connect(ui.actionRun, SIGNAL(triggered()), this, SLOT(onRun()));
	connect(ui.actionRunBackward, SIGNAL(triggered()), this, SLOT(onRunBackward()));
	connect(ui.actionStop, SIGNAL(triggered()), this, SLOT(onStop()));
	connect(ui.actionStepForward, SIGNAL(triggered()), this, SLOT(onStepForward()));
	connect(ui.actionStepBackward, SIGNAL(triggered()), this, SLOT(onStepBackward()));
	connect(ui.actionCollisionCheck, SIGNAL(triggered()), this, SLOT(onCollisionCheck()));
	connect(ui.actionRestrictMotionRange, SIGNAL(triggered()), this, SLOT(onRestrictMotionRange()));
	connect(ui.actionOptions, SIGNAL(triggered()), this, SLOT(onOptions()));
	connect(ui.actionShowSolutions, SIGNAL(triggered()), this, SLOT(onShowSolutions()));
	connect(ui.actionShowGridLines, SIGNAL(triggered()), this, SLOT(onShowGridLines()));
	connect(ui.actionShowInputPoses, SIGNAL(triggered()), this, SLOT(onShowInputPoses()));
	connect(ui.actionRenderBasic, SIGNAL(triggered()), this, SLOT(onRenderingChanged()));
	connect(ui.actionRenderSSAO, SIGNAL(triggered()), this, SLOT(onRenderingChanged()));
	connect(ui.actionRenderLine, SIGNAL(triggered()), this, SLOT(onRenderingChanged()));
	connect(ui.actionRenderHatching, SIGNAL(triggered()), this, SLOT(onRenderingChanged()));

	// create tool bar for file menu
	ui.mainToolBar->addAction(ui.actionNew);
	ui.mainToolBar->addAction(ui.actionOpen);
	ui.mainToolBar->addAction(ui.actionSave);
	ui.mainToolBar->addSeparator();

	// create tool bar for edit menu
	ui.mainToolBar->addAction(ui.actionUndo);
	ui.mainToolBar->addAction(ui.actionRedo);
	ui.mainToolBar->addAction(ui.actionCopy);
	ui.mainToolBar->addAction(ui.actionPaste);
	ui.mainToolBar->addAction(ui.actionDelete);
	ui.mainToolBar->addSeparator();

	// create tool bar for modes
	ui.mainToolBar->addAction(ui.actionSelect);
	ui.mainToolBar->addAction(ui.actionFixedRectangle);
	ui.mainToolBar->addAction(ui.actionFixedCircle);
	ui.mainToolBar->addAction(ui.actionFixedPolygon);
	ui.mainToolBar->addAction(ui.actionMovingRectangle);
	ui.mainToolBar->addAction(ui.actionMovingCircle);
	ui.mainToolBar->addAction(ui.actionMovingPolygon);
	ui.mainToolBar->addAction(ui.actionLinkageRegion);
	ui.mainToolBar->addAction(ui.actionLinkageAvoidance);
	ui.mainToolBar->addSeparator();

	// create tool bar for linkage generation
	ui.mainToolBar->addAction(ui.actionGenerateLinkage);
	ui.mainToolBar->addSeparator();

	// create tool bar for kinematic simulation
	ui.mainToolBar->addAction(ui.actionStepBackward);
	ui.mainToolBar->addAction(ui.actionRun);
	ui.mainToolBar->addAction(ui.actionStop);
	ui.mainToolBar->addAction(ui.actionStepForward);
}

MainWindow::~MainWindow() {
}

void MainWindow::initLayerMenu(int num_layers) {
	for (int i = 0; i < menuLayers.size(); i++) {
		disconnect(menuLayers[i], SIGNAL(triggered()), this, SLOT(onLayerChanged()));
		ui.menuLayer->removeAction(menuLayers[i]);
		groupLayer->removeAction(menuLayers[i]);
		delete menuLayers[i];
	}
	menuLayers.clear();

	for (int i = 0; i < num_layers; i++) {
		menuLayers.push_back(ui.menuLayer->addAction(QString("Layer %1").arg(i + 1)));
		menuLayers[i]->setCheckable(true);
		groupLayer->addAction(menuLayers[i]);
		connect(menuLayers[i], SIGNAL(triggered()), this, SLOT(onLayerChanged()));
	}
	menuLayers[0]->setChecked(true);
}

void MainWindow::keyPressEvent(QKeyEvent *e) {
	glWidget->keyPressEvent(e);
}

void MainWindow::keyReleaseEvent(QKeyEvent* e) {
	glWidget->keyReleaseEvent(e);
}

void MainWindow::onNew() {
	glWidget->clear();
	setWindowTitle("Kinematic Design");
}

void MainWindow::onOpen() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open design file..."), "", tr("Design files (*.xml)"));
	if (filename.isEmpty()) return;

	glWidget->open(filename);
	setWindowTitle("Kinematic Design - " + QFileInfo(filename).fileName());
}

void MainWindow::onSave() {
	QString filename = QFileDialog::getSaveFileName(this, tr("Save design file..."), "", tr("Design files (*.xml)"));
	if (filename.isEmpty()) return;

	glWidget->save(filename);
	setWindowTitle("Kinematic Design - " + QFileInfo(filename).fileName());
}

void MainWindow::onSaveImage() {
	if (!QDir("screenshot").exists()) {
		QDir().mkdir("screenshot");
	}
	QDateTime dateTime = QDateTime().currentDateTime();
	QString str = QString("screenshot/") + dateTime.toString("yyyyMMddhhmmss") + QString(".png");

	glWidget->saveImage(str);
}

void MainWindow::onExportSTL() {
	QString dirname = QFileDialog::getExistingDirectory(0, ("Select Output Folder"), QDir::currentPath());
	if (dirname.isEmpty()) return;
	glWidget->saveSTL(dirname);
}

void MainWindow::onExportSCAD() {
	QString dirname = QFileDialog::getExistingDirectory(0, ("Select Output Folder"), QDir::currentPath());
	if (dirname.isEmpty()) return;
	glWidget->saveSCAD(dirname);
}

void MainWindow::onUndo() {
	glWidget->undo();
}

void MainWindow::onRedo() {
	glWidget->redo();
}

void MainWindow::onCopy() {
	glWidget->copySelectedShapes();
}

void MainWindow::onPaste() {
	glWidget->pasteCopiedShapes();
}

void MainWindow::onDelete() {
	glWidget->deleteSelectedShapes();
}

void MainWindow::onSelectAll() {
	glWidget->selectAll();
}

void MainWindow::onModeChanged() {
	if (ui.actionSelect->isChecked()) {
		glWidget->setMode(GLWidget3D::MODE_SELECT);
	}
	else if (ui.actionFixedRectangle->isChecked()) {
		glWidget->setMode(GLWidget3D::MODE_FIXED_RECTANGLE);
	}
	else if (ui.actionFixedCircle->isChecked()) {
		glWidget->setMode(GLWidget3D::MODE_FIXED_CIRCLE);
	}
	else if (ui.actionFixedPolygon->isChecked()) {
		glWidget->setMode(GLWidget3D::MODE_FIXED_POLYGON);
	}
	else if (ui.actionMovingRectangle->isChecked()) {
		glWidget->setMode(GLWidget3D::MODE_MOVING_RECTANGLE);
	}
	else if (ui.actionMovingCircle->isChecked()) {
		glWidget->setMode(GLWidget3D::MODE_MOVING_CIRCLE);
	}
	else if (ui.actionMovingPolygon->isChecked()) {
		glWidget->setMode(GLWidget3D::MODE_MOVING_POLYGON);
	}
	else if (ui.actionLinkageRegion->isChecked()) {
		glWidget->setMode(GLWidget3D::MODE_LINKAGE_REGION);
	}
	else if (ui.actionLinkageAvoidance->isChecked()) {
		glWidget->setMode(GLWidget3D::MODE_LINKAGE_AVOIDANCE);
	}
	else if (ui.actionKinematics->isChecked()) {
		glWidget->setMode(GLWidget3D::MODE_KINEMATICS);
	}
	update();
}

void MainWindow::onAddLayer() {
	menuLayers.push_back(ui.menuLayer->addAction(QString("Layer %1").arg(menuLayers.size() + 1)));
	menuLayers.back()->setCheckable(true);
	groupLayer->addAction(menuLayers.back());
	menuLayers.back()->setChecked(true);
	connect(menuLayers.back(), SIGNAL(triggered()), this, SLOT(onLayerChanged()));

	glWidget->addLayer();
}

void MainWindow::onInsertLayer() {
	menuLayers.push_back(ui.menuLayer->addAction(QString("Layer %1").arg(menuLayers.size() + 1)));
	menuLayers.back()->setCheckable(true);
	groupLayer->addAction(menuLayers.back());
	connect(menuLayers.back(), SIGNAL(triggered()), this, SLOT(onLayerChanged()));

	glWidget->insertLayer();
}

void MainWindow::onDeleteLayer() {
	disconnect(menuLayers.back(), SIGNAL(triggered()), this, SLOT(onLayerChanged()));
	ui.menuLayer->removeAction(menuLayers.back());
	groupLayer->removeAction(menuLayers.back());
	delete menuLayers.back();
	menuLayers.resize(menuLayers.size() - 1);

	glWidget->deleteLayer();
}

void MainWindow::onLayerChanged() {
	for (int i = 0; i < menuLayers.size(); i++) {
		if (menuLayers[i]->isChecked()) {
			glWidget->setLayer(i);
			break;
		}
	}
}

void MainWindow::onGenerateLinkage() {
	LinkageSynthesisOptionDialog dlg;
	if (dlg.exec()) {
		int linkage_type = 0;
		if (dlg.ui.checkBox4RLinkage->isChecked()) linkage_type |= 1;
		if (dlg.ui.checkBoxSliderCrank->isChecked()) linkage_type |= 2;
		if (linkage_type == 0) {
			QMessageBox msg(this);
			msg.setWindowTitle("Error");
			msg.setText("Please select at least one linkage type.");
			msg.exec();
			return;
		}

		std::vector<std::pair<double, double>> sigmas = {
			std::make_pair(dlg.ui.lineEditStdDevPositionFirst->text().toDouble(), dlg.ui.lineEditStdDevOrientationFirst->text().toDouble()),
			std::make_pair(dlg.ui.lineEditStdDevPositionMiddle->text().toDouble(), dlg.ui.lineEditStdDevOrientationMiddle->text().toDouble()),
			std::make_pair(dlg.ui.lineEditStdDevPositionLast->text().toDouble(), dlg.ui.lineEditStdDevOrientationLast->text().toDouble())
		};

		std::vector<double> weights = {
			dlg.ui.lineEditPositionErrorWeight->text().toDouble(),
			dlg.ui.lineEditOrientationErrorWeight->text().toDouble(),
			dlg.ui.lineEditLinkageLocationWeight->text().toDouble(),
			dlg.ui.lineEditTrajectoryWeight->text().toDouble(),
			dlg.ui.lineEditSizeWeight->text().toDouble(),
			dlg.ui.lineEditLinkageDepthWeight->text().toDouble()
		};

		glWidget->calculateSolutions(linkage_type,
			dlg.ui.lineEditNumSamples->text().toInt(),
			sigmas,
			dlg.ui.checkBoxAvoidBranchDefect->isChecked(),
			dlg.ui.lineEditMinTransmissionAngle->text().toDouble(),
			weights,
			dlg.ui.lineEditNumParticles->text().toInt(),
			dlg.ui.lineEditNumIterations->text().toInt(),
			dlg.ui.checkBoxRecordFile->isChecked());
	}
}

void MainWindow::onRun() {
	glWidget->run();
}

void MainWindow::onRunBackward() {
	glWidget->invertSpeed();
	glWidget->run();
}

void MainWindow::onStop() {
	glWidget->stop();
}

void MainWindow::onStepForward() {
	glWidget->stepForward();
}

void MainWindow::onStepBackward() {
	glWidget->stepBackward();
}

void MainWindow::onCollisionCheck() {
	glWidget->collision_check = ui.actionCollisionCheck->isChecked();
}

void MainWindow::onRestrictMotionRange() {
	glWidget->restrict_motion_range = ui.actionRestrictMotionRange->isChecked();
}

void MainWindow::onOptions() {
	OptionDialog dlg;
	dlg.setBodyMargin(kinematics::options->body_margin);
	dlg.setGap(kinematics::options->gap);
	dlg.setLinkWidth(kinematics::options->link_width);
	dlg.setLinkDepth(kinematics::options->link_depth);
	dlg.setHoleRadius(kinematics::options->hole_radius);
	dlg.setHoleRadius(kinematics::options->hole_radius);
	dlg.setJointRaidus(kinematics::options->joint_radius);
	dlg.setJointCapRaidus1(kinematics::options->joint_cap_radius1);
	dlg.setJointCapRaidus2(kinematics::options->joint_cap_radius2);
	dlg.setJointCapDepth(kinematics::options->joint_cap_depth);
	dlg.setSliderGuideWidth(kinematics::options->slider_guide_width);
	dlg.setSliderGuideDepth(kinematics::options->slider_guide_depth);
	dlg.setBodyDepth(kinematics::options->body_depth);

	if (dlg.exec()) {
		kinematics::options->body_margin = dlg.getBodyMargin();
		kinematics::options->gap = dlg.getGap();
		kinematics::options->link_width = dlg.getLinkWidth();
		kinematics::options->link_depth = dlg.getLinkDepth();
		kinematics::options->hole_radius = dlg.getHoleRadius();
		kinematics::options->joint_radius = dlg.getJointRadius();
		kinematics::options->joint_cap_radius1 = dlg.getJointCapRadius1();
		kinematics::options->joint_cap_radius2 = dlg.getJointCapRadius2();
		kinematics::options->joint_cap_depth = dlg.getJointCapDepth();
		kinematics::options->slider_guide_width = dlg.getSliderGuideWidth();
		kinematics::options->slider_guide_depth = dlg.getSliderGuideDepth();
		kinematics::options->body_depth = dlg.getBodyDepth();
	}
}

void MainWindow::onShowSolutions() {
	glWidget->show_solutions = ui.actionShowSolutions->isChecked();
	update();
}

void MainWindow::onShowGridLines() {
	glWidget->show_grid_lines = ui.actionShowGridLines->isChecked();
	update();
}

void MainWindow::onShowInputPoses() {
	glWidget->show_input_poses = ui.actionShowInputPoses->isChecked();
	update();
}

void MainWindow::onRenderingChanged() {
	if (ui.actionRenderBasic->isChecked()) {
		glWidget->renderManager.renderingMode = RenderManager::RENDERING_MODE_BASIC;
	}
	else if (ui.actionRenderSSAO->isChecked()) {
		glWidget->renderManager.renderingMode = RenderManager::RENDERING_MODE_SSAO;
	}
	else if (ui.actionRenderLine->isChecked()) {
		glWidget->renderManager.renderingMode = RenderManager::RENDERING_MODE_LINE;
	}
	else if (ui.actionRenderHatching->isChecked()) {
		glWidget->renderManager.renderingMode = RenderManager::RENDERING_MODE_HATCHING;
	}
}