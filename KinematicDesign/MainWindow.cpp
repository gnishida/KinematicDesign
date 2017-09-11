#include "MainWindow.h"
#include <QFileDialog>
#include "LinkageSynthesisOptionDialog.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	ui.setupUi(this);

	glWidget = new GLWidget3D(this);
	setCentralWidget(glWidget);

	QActionGroup* groupMode = new QActionGroup(this);
	groupMode->addAction(ui.actionSelect);
	groupMode->addAction(ui.actionRectangle);
	groupMode->addAction(ui.actionCircle);
	groupMode->addAction(ui.actionPolygon);
	groupMode->addAction(ui.actionLinkageRegion);
	groupMode->addAction(ui.actionKinematics);
	ui.actionSelect->setChecked(true);

	groupLayer = new QActionGroup(this);
	initLayerMenu(2);

	ui.actionCollisionCheck->setChecked(glWidget->collision_check);

	connect(ui.actionNew, SIGNAL(triggered()), this, SLOT(onNew()));
	connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onOpen()));
	connect(ui.actionSave, SIGNAL(triggered()), this, SLOT(onSave()));
	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.actionUndo, SIGNAL(triggered()), this, SLOT(onUndo()));
	connect(ui.actionRedo, SIGNAL(triggered()), this, SLOT(onRedo()));
	connect(ui.actionCopy, SIGNAL(triggered()), this, SLOT(onCopy()));
	connect(ui.actionPaste, SIGNAL(triggered()), this, SLOT(onPaste()));
	connect(ui.actionDelete, SIGNAL(triggered()), this, SLOT(onDelete()));
	connect(ui.actionSelectAll, SIGNAL(triggered()), this, SLOT(onSelectAll()));
	connect(ui.actionSelect, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionRectangle, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionCircle, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionPolygon, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionLinkageRegion, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionKinematics, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionAddLayer, SIGNAL(triggered()), this, SLOT(onAddLayer()));
	connect(ui.actionInsertLayer, SIGNAL(triggered()), this, SLOT(onInsertLayer()));
	connect(ui.actionDeleteLayer, SIGNAL(triggered()), this, SLOT(onDeleteLayer()));
	connect(ui.actionCalculateSolution4RLinkage, SIGNAL(triggered()), this, SLOT(onCalculateSolution4RLinkage()));
	connect(ui.actionCalculateSolutionSliderCrank, SIGNAL(triggered()), this, SLOT(onCalculateSolutionSliderCrank()));
	connect(ui.actionRun, SIGNAL(triggered()), this, SLOT(onRun()));
	connect(ui.actionRunBackward, SIGNAL(triggered()), this, SLOT(onRunBackward()));
	connect(ui.actionStop, SIGNAL(triggered()), this, SLOT(onStop()));
	connect(ui.actionStepForward, SIGNAL(triggered()), this, SLOT(onStepForward()));
	connect(ui.actionStepBackward, SIGNAL(triggered()), this, SLOT(onStepBackward()));
	connect(ui.actionCollisionCheck, SIGNAL(triggered()), this, SLOT(onCollisionCheck()));
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

void MainWindow::onNew() {
	glWidget->clear();
	setWindowTitle("Canvas 3D");
}

void MainWindow::onOpen() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open design file..."), "", tr("Design files (*.xml)"));
	if (filename.isEmpty()) return;

	glWidget->open(filename);
	setWindowTitle("Canvas 3D - " + QFileInfo(filename).fileName());
}

void MainWindow::onSave() {
	QString filename = QFileDialog::getSaveFileName(this, tr("Save design file..."), "", tr("Design files (*.xml)"));
	if (filename.isEmpty()) return;

	glWidget->save(filename);
	setWindowTitle("Canvas 3D - " + QFileInfo(filename).fileName());
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

void MainWindow::keyPressEvent(QKeyEvent *e) {
	glWidget->keyPressEvent(e);
}

void MainWindow::keyReleaseEvent(QKeyEvent* e) {
	glWidget->keyReleaseEvent(e);
}

void MainWindow::onModeChanged() {
	if (ui.actionSelect->isChecked()) {
		glWidget->setMode(GLWidget3D::MODE_SELECT);
	}
	else if (ui.actionRectangle->isChecked()) {
		glWidget->setMode(GLWidget3D::MODE_RECTANGLE);
	}
	else if (ui.actionCircle->isChecked()) {
		glWidget->setMode(GLWidget3D::MODE_CIRCLE);
	}
	else if (ui.actionPolygon->isChecked()) {
		glWidget->setMode(GLWidget3D::MODE_POLYGON);
	}
	else if (ui.actionLinkageRegion->isChecked()) {
		glWidget->setMode(GLWidget3D::MODE_LINKAGE_REGION);
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

void MainWindow::onCalculateSolution4RLinkage() {
	LinkageSynthesisOptionDialog dlg;
	if (dlg.exec()) {
		std::vector<std::pair<double, double>> sigmas = {
			std::make_pair(dlg.ui.lineEditStdDevPositionFirst->text().toDouble(), dlg.ui.lineEditStdDevOrientationFirst->text().toDouble()),
			std::make_pair(dlg.ui.lineEditStdDevPositionMiddle->text().toDouble(), dlg.ui.lineEditStdDevOrientationMiddle->text().toDouble()),
			std::make_pair(dlg.ui.lineEditStdDevPositionLast->text().toDouble(), dlg.ui.lineEditStdDevOrientationLast->text().toDouble())
		};

		glWidget->calculateSolutions(GLWidget3D::LINKAGE_4R,
			dlg.ui.lineEditNumSamples->text().toInt(),
			sigmas,
			dlg.ui.checkBoxAvoidBranchDefect->isChecked(),
			dlg.ui.checkBoxRotatableCrank->isChecked(),
			dlg.ui.lineEditPositionErrorWeight->text().toDouble(),
			dlg.ui.lineEditOrientationErrorWeight->text().toDouble(),
			dlg.ui.lineEditLinkageLocationWeight->text().toDouble(),
			dlg.ui.lineEditTrajectoryWeight->text().toDouble(),
			dlg.ui.lineEditSizeWeight->text().toDouble());
	}
}

void MainWindow::onCalculateSolutionSliderCrank() {
	LinkageSynthesisOptionDialog dlg;
	if (dlg.exec()) {
		std::vector<std::pair<double, double>> sigmas = {
			std::make_pair(dlg.ui.lineEditStdDevPositionFirst->text().toDouble(), dlg.ui.lineEditStdDevOrientationFirst->text().toDouble()),
			std::make_pair(dlg.ui.lineEditStdDevPositionMiddle->text().toDouble(), dlg.ui.lineEditStdDevOrientationMiddle->text().toDouble()),
			std::make_pair(dlg.ui.lineEditStdDevPositionLast->text().toDouble(), dlg.ui.lineEditStdDevOrientationLast->text().toDouble())
		};

		glWidget->calculateSolutions(GLWidget3D::LINKAGE_RRRP,
			dlg.ui.lineEditNumSamples->text().toInt(),
			sigmas,
			dlg.ui.checkBoxAvoidBranchDefect->isChecked(),
			dlg.ui.checkBoxRotatableCrank->isChecked(),
			dlg.ui.lineEditPositionErrorWeight->text().toDouble(),
			dlg.ui.lineEditOrientationErrorWeight->text().toDouble(),
			dlg.ui.lineEditLinkageLocationWeight->text().toDouble(),
			dlg.ui.lineEditTrajectoryWeight->text().toDouble(),
			dlg.ui.lineEditSizeWeight->text().toDouble());
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