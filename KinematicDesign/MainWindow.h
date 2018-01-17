#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include "GLWidget3D.h"

class MainWindow : public QMainWindow {
	Q_OBJECT

public:
	Ui::MainWindowClass ui;
	std::vector<QAction*> menuLayers;
	QActionGroup* groupLayer;
	QActionGroup* groupRender;
	GLWidget3D* glWidget;

public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();

	void initLayerMenu(int num_layers);

protected:
	void keyPressEvent(QKeyEvent* e);
	void keyReleaseEvent(QKeyEvent* e);

public slots:
	void onNew();
	void onOpen();
	void onSave();
	void onSaveImage();
	void onExportSTL();
	void onExportSCAD();
	void onUndo();
	void onRedo();
	void onCopy();
	void onPaste();
	void onDelete();
	void onSelectAll();
	void onModeChanged();
	void onAddLayer();
	void onInsertLayer();
	void onDeleteLayer();
	void onLayerChanged();
	void onGenerateLinkage();
	void onRun();
	void onRunBackward();
	void onStop();
	void onStepForward();
	void onStepBackward();
	void onCollisionCheck();
	void onRestrictMotionRange();
	void onOptions();
	void onShowSolutions();
	void onShowGridLines();
	void onShowInputPoses();
	void onRenderingChanged();
};

#endif // MAINWINDOW_H
