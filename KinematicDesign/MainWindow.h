#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include "Canvas.h"
#include <vector>

class MainWindow : public QMainWindow {
	Q_OBJECT

public:
	Ui::MainWindowClass ui;
	std::vector<QAction*> menuLayers;
	QActionGroup* groupLayer;
	canvas::Canvas* canvas;

public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();

	void initLayerMenu(int num_layers);

public slots:
	void onNew();
	void onOpen();
	void onSave();
	void onUndo();
	void onRedo();
	void onCopy();
	void onPaste();
	void onDelete();
	void onSelectAll();
	void onCircularRepeat();
	void onModeChanged();
	void onAddLayer();
	void onInsertLayer();
	void onDeleteLayer();
	void onLayerChanged();
	void onCalculateSolution4RLinkage();
	void onCalculateSolutionSliderCrank();
	void onRun();
	void onRunBackward();
	void onStop();
	void onStepForward();
	void onStepBackward();
	void onCollisionCheck();
	void keyPressEvent(QKeyEvent* e);
	void keyReleaseEvent(QKeyEvent* e);
};

#endif // MAINWINDOW_H
