#include "MainWindow.h"
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	ui.setupUi(this);

	QActionGroup* groupMode = new QActionGroup(this);
	groupMode->addAction(ui.actionMove);
	groupMode->addAction(ui.actionRectangle);
	groupMode->addAction(ui.actionPolygon);
	ui.actionMove->setChecked(true);

	QActionGroup* groupLayer = new QActionGroup(this);
	groupLayer->addAction(ui.actionLayer1);
	groupLayer->addAction(ui.actionLayer2);
	ui.actionLayer1->setChecked(true);

	canvas = new canvas::Canvas(this);
	setCentralWidget(canvas);

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
	connect(ui.actionMove, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionRectangle, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionPolygon, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionLayer1, SIGNAL(triggered()), this, SLOT(onLayerChanged()));
	connect(ui.actionLayer2, SIGNAL(triggered()), this, SLOT(onLayerChanged()));
	connect(ui.actionInitialKinematicDiagram, SIGNAL(triggered()), this, SLOT(onInitialKinematicDiagram()));
	connect(ui.actionSolveInverse, SIGNAL(triggered()), this, SLOT(onSolveInverse()));
	connect(ui.actionRun, SIGNAL(triggered()), this, SLOT(onRun()));
	connect(ui.actionStop, SIGNAL(triggered()), this, SLOT(onStop()));
	connect(ui.actionStepForward, SIGNAL(triggered()), this, SLOT(onStepForward()));
	connect(ui.actionStepBackward, SIGNAL(triggered()), this, SLOT(onStepBackward()));
}

MainWindow::~MainWindow() {
}

void MainWindow::keyPressEvent(QKeyEvent* e) {
	canvas->keyPressEvent(e);
}

void MainWindow::keyReleaseEvent(QKeyEvent* e) {
	canvas->keyReleaseEvent(e);
}

void MainWindow::onNew() {
	canvas->clear();
}

void MainWindow::onOpen() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open Design file..."), "", tr("Design Files (*.xml)"));
	if (filename.isEmpty()) return;

	canvas->open(filename);
}

void MainWindow::onSave() {
	QString filename = QFileDialog::getSaveFileName(this, tr("Save Design file..."), "", tr("Design Files (*.xml)"));
	if (filename.isEmpty()) return;

	canvas->save(filename);
}

void MainWindow::onUndo() {
	canvas->undo();
}

void MainWindow::onRedo() {
	canvas->redo();
}

void MainWindow::onCopy() {
	canvas->copySelectedShapes();
}

void MainWindow::onPaste() {
	canvas->pasteCopiedShapes();
}

void MainWindow::onDelete() {
	canvas->deleteSelectedShapes();
}

void MainWindow::onSelectAll() {
	canvas->selectAll();
}

void MainWindow::onModeChanged() {
	if (ui.actionMove->isChecked()) {
		canvas->setMode(canvas::Canvas::MODE_MOVE);
	}
	else if (ui.actionRectangle->isChecked()) {
		canvas->setMode(canvas::Canvas::MODE_RECTANGLE);
	}
	else if (ui.actionPolygon->isChecked()) {
		canvas->setMode(canvas::Canvas::MODE_POLYGON);
	}
	update();
}

void MainWindow::onLayerChanged() {
	if (ui.actionLayer1->isChecked()) {
		canvas->setLayer(0);
	}
	else if (ui.actionLayer2->isChecked()) {
		canvas->setLayer(1);
	}
}

void MainWindow::onInitialKinematicDiagram() {
	canvas->initialKinematicDiagram();
}

void MainWindow::onSolveInverse() {
	canvas->solveInverse();
}

void MainWindow::onRun() {
	canvas->run();
}

void MainWindow::onStop() {
	canvas->stop();
}

void MainWindow::onStepForward() {
	canvas->stepForward();
}

void MainWindow::onStepBackward() {
	canvas->stepBackward();
}