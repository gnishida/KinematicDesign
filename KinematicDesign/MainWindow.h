#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include "Canvas.h"

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	Ui::MainWindowClass ui;
	canvas::Canvas* canvas;

public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();

protected:
	void keyPressEvent(QKeyEvent* e);
	void keyReleaseEvent(QKeyEvent* e);

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
	void onModeChanged();
	void onLayerChanged();
};

#endif // MAINWINDOW_H
