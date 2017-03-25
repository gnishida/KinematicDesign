#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	ui.setupUi(this);

	QActionGroup* groupMode = new QActionGroup(this);
	groupMode->addAction(ui.actionMove);
	groupMode->addAction(ui.actionRectangle);
	groupMode->addAction(ui.actionPolygon);
	ui.actionMove->setChecked(true);

	canvas = new canvas::Canvas(this);
	setCentralWidget(canvas);

	connect(ui.actionNew, SIGNAL(triggered()), this, SLOT(onNew()));
	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.actionCopy, SIGNAL(triggered()), this, SLOT(onCopy()));
	connect(ui.actionPaste, SIGNAL(triggered()), this, SLOT(onPaste()));
	connect(ui.actionDelete, SIGNAL(triggered()), this, SLOT(onDelete()));
	connect(ui.actionSelectAll, SIGNAL(triggered()), this, SLOT(onSelectAll()));
	connect(ui.actionMove, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionRectangle, SIGNAL(triggered()), this, SLOT(onModeChanged()));
	connect(ui.actionPolygon, SIGNAL(triggered()), this, SLOT(onModeChanged()));
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