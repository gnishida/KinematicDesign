#include "Canvas.h"
#include <QPainter>
#include <iostream>
#include <QFileInfoList>
#include <QDir>
#include <QMessageBox>
#include <QTextStream>
#include <QDomDocument>
#include <QResizeEvent>
#include <QtWidgets/QApplication>
#include "Rectangle.h"
#include "Polygon.h"
#include "MainWindow.h"

namespace canvas {

	Canvas::Canvas(MainWindow* mainWin) : QWidget((QWidget*)mainWin) {
		this->mainWin = mainWin;
		ctrlPressed = false;
		shiftPressed = false;

		mode = MODE_MOVE;
		drawing_shape = false;

		animation_timer = NULL;
		simulation_speed = 0.01;
	}

	Canvas::~Canvas() {
	}

	void Canvas::clear() {
		shapes.clear();
		update();
	}

	void Canvas::selectAll() {
		for (int i = 0; i < shapes.size(); ++i) {
			shapes[i]->select();
		}
		mode = MODE_MOVE;
		update();
	}

	void Canvas::unselectAll() {
		for (int i = 0; i < shapes.size(); ++i) {
			shapes[i]->unselect();
		}
		update();
	}

	void Canvas::deleteSelectedShapes() {
		for (int i = shapes.size() - 1; i >= 0; --i) {
			if (shapes[i]->isSelected()) {
				shapes.erase(shapes.begin() + i);
			}
		}
		update();
	}

	void Canvas::copySelectedShapes() {
		copied_shapes.clear();
		for (int i = 0; i < shapes.size(); ++i) {
			if (shapes[i]->isSelected()) {
				copied_shapes.push_back(shapes[i]->clone());
			}
		}
	}

	void Canvas::pasteCopiedShapes() {
		unselectAll();
		for (int i = 0; i < copied_shapes.size(); ++i) {
			boost::shared_ptr<Shape> shape = copied_shapes[i]->clone();
			shape->select();
			shapes.push_back(shape);
		}

		mode = MODE_MOVE;
		update();
	}

	void Canvas::setMode(int mode) {
		this->mode = mode;
		unselectAll();
		update();
	}

	void Canvas::open(const QString& filename) {
		kinematics.load(filename);

		// Since the direction of the speed might be inverted due to the dead zone,
		// we need to recover the original speed when a new object is loaded.
		simulation_speed = 0.01;

		update();
	}

	void Canvas::save(const QString& filename) {
		kinematics.save(filename);
	}

	void Canvas::run() {
		if (animation_timer == NULL) {
			animation_timer = new QTimer(this);
			connect(animation_timer, SIGNAL(timeout()), this, SLOT(animation_update()));
			animation_timer->start(10);
		}
	}

	void Canvas::stop() {
		if (animation_timer != NULL) {
			animation_timer->stop();
			delete animation_timer;
			animation_timer = NULL;
		}
	}

	void Canvas::speedUp() {
		simulation_speed *= 2;
	}

	void Canvas::speedDown() {
		simulation_speed *= 0.5;
	}

	void Canvas::invertSpeed() {
		simulation_speed = -simulation_speed;
	}

	void Canvas::stepForward() {
		if (animation_timer == NULL) {
			try {
				kinematics.stepForward(simulation_speed);
			}
			catch (char* ex) {
				simulation_speed = -simulation_speed;
				std::cerr << "Animation is stopped by error:" << std::endl;
				std::cerr << ex << std::endl;
			}
			update();
		}
	}

	void Canvas::stepBackward() {
		if (animation_timer == NULL) {
			try {
				kinematics.stepForward(-simulation_speed);
			}
			catch (char* ex) {
				simulation_speed = -simulation_speed;
				std::cerr << "Animation is stopped by error:" << std::endl;
				std::cerr << ex << std::endl;
			}
			update();
		}
	}

	void Canvas::showAssemblies(bool flag) {
		kinematics.showAssemblies(flag);
		update();
	}

	void Canvas::showLinks(bool flag) {
		kinematics.showLinks(flag);
		update();
	}

	void Canvas::showBodies(bool flag) {
		kinematics.showBodies(flag);
		update();
	}

	void Canvas::animation_update() {
		try {
			kinematics.stepForward(simulation_speed);
		}
		catch (char* ex) {
			simulation_speed = -simulation_speed;
			//stop();
			std::cerr << "Animation is stopped by error:" << std::endl;
			std::cerr << ex << std::endl;
		}

		update();
	}

	void Canvas::paintEvent(QPaintEvent *e) {
		QPainter painter(this);

		for (int i = 0; i < shapes.size(); ++i) {
			shapes[i]->draw(painter);
		}

		if (mode == MODE_RECTANGLE || mode == MODE_POLYGON) {
			if (drawing_shape) {
				current_shape->draw(painter);
			}
		}

		kinematics.draw(painter);
	}

	void Canvas::mousePressEvent(QMouseEvent* e) {
		// This is necessary to get key event occured even after the user selects a menu.
		setFocus();

		if (mode == MODE_MOVE) {
			prev_mouse_pt = glm::dvec2(e->x(), e->y());

			// hit test for rotation marker
			for (int i = 0; i < shapes.size(); ++i) {
				if (glm::length(shapes[i]->getRotationMarkerPosition() - shapes[i]->localCoordinate(glm::dvec2(e->x(), e->y()))) < 10) {
					mode = MODE_ROTATION;
					selected_shape = shapes[i];
					if (!shapes[i]->isSelected()) {
						unselectAll();
						shapes[i]->select();
					}
					update();
					return;
				}
			}

			// hit test for resize marker
			for (int i = 0; i < shapes.size(); ++i) {
				BoundingBox bbox = shapes[i]->boundingBox();
				if (glm::length(bbox.minPt - shapes[i]->localCoordinate(glm::dvec2(e->x(), e->y()))) < 10) {
					mode = MODE_RESIZE_TOP_LEFT;
					selected_shape = shapes[i];
					if (!shapes[i]->isSelected()) {
						unselectAll();
						shapes[i]->select();
					}
					update();
					return;
				}

				if (glm::length(bbox.maxPt - shapes[i]->localCoordinate(glm::dvec2(e->x(), e->y()))) < 10) {
					mode = MODE_RESIZE_BOTTOM_RIGHT;
					selected_shape = shapes[i];
					if (!shapes[i]->isSelected()) {
						unselectAll();
						shapes[i]->select();
					}
					update();
					return;
				}
			}

			// hit test for the shape
			for (int i = 0; i < shapes.size(); ++i) {
				if (shapes[i]->hit(glm::dvec2(e->x(), e->y()))) {
					if (!shapes[i]->isSelected()) {
						if (!ctrlPressed) {
							// If ctrl is not pressed, then deselect all other shapes.
							unselectAll();
						}
						shapes[i]->select();
					}
					update();
					return;
				}
			}

			unselectAll();
		}
		else if (mode == MODE_RECTANGLE) {
			if (drawing_shape) {
				// do nothing
			}
			else {
				// start drawing a rectangle
				unselectAll();
				drawing_shape = true;
				current_shape = boost::shared_ptr<Shape>(new Rectangle(glm::dvec2(e->x(), e->y())));
				setMouseTracking(true);
			}
		}
		else if (mode == MODE_POLYGON) {
			if (drawing_shape) {
				current_shape->addPoint(current_shape->localCoordinate(glm::dvec2(e->x(), e->y())));
			}
			else {
				// start drawing a polygon
				unselectAll();
				drawing_shape = true;
				current_shape = boost::shared_ptr<Shape>(new Polygon(glm::dvec2(e->x(), e->y())));
				setMouseTracking(true);
			}
		}

		update();
	}

	void Canvas::mouseMoveEvent(QMouseEvent* e) {
		if (mode == MODE_MOVE) {
			glm::dvec2 dir = glm::dvec2(e->x(), e->y()) - prev_mouse_pt;
			for (int i = 0; i < shapes.size(); ++i) {
				if (shapes[i]->isSelected()) {
					shapes[i]->translate(dir);
				}
			}
			prev_mouse_pt = glm::dvec2(e->x(), e->y());
			update();
		}
		else if (mode == MODE_ROTATION) {
			glm::dvec2 dir1 = prev_mouse_pt - selected_shape->worldCoordinate(selected_shape->getCenter());
			glm::dvec2 dir2 = glm::dvec2(e->x(), e->y()) - selected_shape->worldCoordinate(selected_shape->getCenter());
			double theta = atan2(dir2.y, dir2.x) - atan2(dir1.y, dir1.x);
			for (int i = 0; i < shapes.size(); ++i) {
				if (shapes[i]->isSelected()) {
					shapes[i]->rotate(theta);
				}
			}
			prev_mouse_pt = glm::dvec2(e->x(), e->y());
			update();
		}
		else if (mode == MODE_RESIZE_TOP_LEFT) {
			glm::dvec2 dir1 = selected_shape->boundingBox().minPt - selected_shape->boundingBox().maxPt;
			glm::dvec2 dir2 = selected_shape->localCoordinate(glm::dvec2(e->x(), e->y())) - selected_shape->boundingBox().maxPt;
			glm::dvec2 scale(dir2.x / dir1.x, dir2.y / dir1.y);
			for (int i = 0; i < shapes.size(); ++i) {
				if (shapes[i]->isSelected()) {
					shapes[i]->resize(scale, Shape::RESIZE_TOP_LEFT);
				}
			}
			prev_mouse_pt = glm::dvec2(e->x(), e->y());
			update();
		}
		else if (mode == MODE_RESIZE_BOTTOM_RIGHT) {
			glm::dvec2 dir1 = selected_shape->boundingBox().maxPt - selected_shape->boundingBox().minPt;
			glm::dvec2 dir2 = selected_shape->localCoordinate(glm::dvec2(e->x(), e->y())) - selected_shape->boundingBox().minPt;
			glm::dvec2 scale(dir2.x / dir1.x, dir2.y / dir1.y);
			for (int i = 0; i < shapes.size(); ++i) {
				if (shapes[i]->isSelected()) {
					shapes[i]->resize(scale, Shape::RESIZE_BOTTOM_RIGHT);
				}
			}
			prev_mouse_pt = glm::dvec2(e->x(), e->y());
			update();
		}
		else if (mode == MODE_RECTANGLE || mode == MODE_POLYGON) {
			if (drawing_shape) {
				current_shape->updateByNewPoint(current_shape->localCoordinate(glm::dvec2(e->x(), e->y())));
				update();
			}
		}
	}

	void Canvas::mouseReleaseEvent(QMouseEvent* e) {
		if (mode == MODE_ROTATION || mode == MODE_RESIZE_TOP_LEFT || mode == MODE_RESIZE_TOP_RIGHT || mode == MODE_RESIZE_BOTTOM_LEFT || mode == MODE_RESIZE_BOTTOM_RIGHT) {
			mode = MODE_MOVE;
		}
	}

	void Canvas::mouseDoubleClickEvent(QMouseEvent* e) {
		if (mode == MODE_RECTANGLE) {
			if (drawing_shape) {
				// The polygon is created.
				drawing_shape = false;
				current_shape->complete();
				current_shape->select();
				shapes.push_back(current_shape);
				setMouseTracking(false);
			}
		}
		else if (mode == MODE_POLYGON) {
			if (drawing_shape) {
				// The polygon is created.
				drawing_shape = false;
				current_shape->complete();
				current_shape->select();
				shapes.push_back(current_shape);
				setMouseTracking(false);
			}
		}

		update();
	}

	void Canvas::resizeEvent(QResizeEvent *e) {
	}

	void Canvas::keyPressEvent(QKeyEvent* e) {
		ctrlPressed = false;
		shiftPressed = false;

		if (e->modifiers() & Qt::ControlModifier) {
			ctrlPressed = true;
		}
		if (e->modifiers() & Qt::ShiftModifier) {
			shiftPressed = true;
		}

		switch (e->key()) {
		case Qt::Key_Escape:
			if (mode == MODE_RECTANGLE || mode == MODE_POLYGON) {
				drawing_shape = false;
				update();
			}
			break;
		case Qt::Key_Space:
			break;
		case Qt::Key_Delete:
			break;
		}

		update();
	}

	void Canvas::keyReleaseEvent(QKeyEvent* e) {
		switch (e->key()) {
		case Qt::Key_Control:
			ctrlPressed = false;
			break;
		case Qt::Key_Shift:
			shiftPressed = false;
			break;
		default:
			break;
		}
	}

}