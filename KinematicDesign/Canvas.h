#ifndef CANVAS_H
#define CANVAS_H

#include <QWidget>
#include <QKeyEvent>
#include <glm/glm.hpp>
#include <boost/shared_ptr.hpp>
#include <Kinematics.h>
#include "Utils.h"
#include "Shape.h"
#include "Layer.h"
#include <QTimer>

class MainWindow;

namespace canvas {
	
	class Canvas : public QWidget {
		Q_OBJECT

	public:
		static enum { MODE_MOVE = 0, MODE_ROTATION, MODE_RESIZE_TOP_LEFT, MODE_RESIZE_TOP_RIGHT, MODE_RESIZE_BOTTOM_LEFT, MODE_RESIZE_BOTTOM_RIGHT, MODE_POLYGON, MODE_RECTANGLE, MODE_CIRCLE };

	private:
		MainWindow* mainWin;
		bool ctrlPressed;
		bool shiftPressed;

		int mode;
		glm::dvec2 prev_mouse_pt;
		boost::shared_ptr<canvas::Shape> current_shape;
		std::vector<Layer> layers;
		int layer_id;
		boost::shared_ptr<canvas::Shape> selected_shape;
		std::vector<boost::shared_ptr<canvas::Shape>> copied_shapes;
		
		kinematics::Kinematics kinematics;
		QTimer* animation_timer;
		float simulation_speed;

	public:
		Canvas(MainWindow* mainWin);
		~Canvas();

		void clear();
		void selectAll();
		void unselectAll();
		void deleteSelectedShapes();
		void copySelectedShapes();
		void pasteCopiedShapes();
		void setMode(int mode);
		void setLayer(int layer_id);
		void open(const QString& filename);
		void save(const QString& filename);
		void run();
		void stop();
		void speedUp();
		void speedDown();
		void invertSpeed();
		void stepForward();
		void stepBackward();
		void showAssemblies(bool flag);
		void showLinks(bool flag);
		void showBodies(bool flag);

		public slots:
		void animation_update();

	protected:
		void paintEvent(QPaintEvent* e);
		void mousePressEvent(QMouseEvent* e);
		void mouseMoveEvent(QMouseEvent* e);
		void mouseReleaseEvent(QMouseEvent* e);
		void mouseDoubleClickEvent(QMouseEvent* e);
		void resizeEvent(QResizeEvent *e);

	public:
		void keyPressEvent(QKeyEvent* e);
		void keyReleaseEvent(QKeyEvent* e);
	};

}

#endif // CANVAS_H
