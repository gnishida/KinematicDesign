#include "Canvas.h"
#include <QPainter>
#include <iostream>
#include <QFileInfoList>
#include <QDir>
#include <QMessageBox>
#include <QTextStream>
#include <QDate>
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
		layers.resize(2);
		layer_id = 0;

		animation_timer = NULL;
		simulation_speed = 0.01;
	}

	Canvas::~Canvas() {
	}

	void Canvas::clear() {
		layers[layer_id].clear();
		update();
	}

	void Canvas::selectAll() {
		layers[layer_id].selectAll();
		mode = MODE_MOVE;
		update();
	}

	void Canvas::unselectAll() {
		layers[layer_id].unselectAll();
		current_shape.reset();
		update();
	}

	void Canvas::deleteSelectedShapes() {
		layers[layer_id].deleteSelectedShapes();
		current_shape.reset();
		update();
	}

	void Canvas::copySelectedShapes() {
		layers[layer_id].copySelectedShapes(copied_shapes);
	}

	void Canvas::pasteCopiedShapes() {
		layers[layer_id].pasteCopiedShapes(copied_shapes);
		current_shape.reset();
		mode = MODE_MOVE;
		update();
	}

	void Canvas::setMode(int mode) {
		this->mode = mode;
		unselectAll();
		update();
	}

	void Canvas::setLayer(int layer_id) {
		if (this->layer_id != layer_id) {
			layers[this->layer_id].unselectAll();
			this->layer_id = layer_id;
			current_shape.reset();
			update();
		}
	}

	void Canvas::open(const QString& filename) {
		QFile file(filename);
		if (!file.open(QFile::ReadOnly | QFile::Text)) throw "Fild cannot open.";

		QDomDocument doc;
		doc.setContent(&file);

		QDomElement root = doc.documentElement();
		if (root.tagName() != "design")	throw "Invalid file format.";

		// clear the data
		layers.clear();
		selected_shape.reset();
		mode = MODE_MOVE;

		QDomNode node = root.firstChild();
		while (!node.isNull()) {
			if (node.toElement().tagName() == "layers") {
				QDomNode layer_node = node.firstChild();
				while (!layer_node.isNull()) {
					if (layer_node.toElement().tagName() == "layer") {
						Layer layer;
						layer.load(layer_node.toElement());
						layers.push_back(layer);
					}

					layer_node = layer_node.nextSibling();
				}
			}

			node = node.nextSibling();
		}

		// select 1st layer to display
		layer_id = 0;

		// no currently drawing shape
		current_shape.reset();

		update();
	}

	void Canvas::save(const QString& filename) {
		QFile file(filename);
		if (!file.open(QFile::WriteOnly)) throw "File cannot open.";

		QDomDocument doc;

		// set root node
		QDomElement root = doc.createElement("design");
		root.setAttribute("author", "Gen Nishida");
		root.setAttribute("version", "1.0");
		root.setAttribute("date", QDate::currentDate().toString("MM/dd/yyyy"));
		doc.appendChild(root);

		// write layers
		QDomElement layers_node = doc.createElement("layers");
		root.appendChild(layers_node);
		for (int i = 0; i < layers.size(); ++i) {
			QDomElement layer_node = layers[i].toXml(doc);
			layers_node.appendChild(layer_node);
		}

		QTextStream out(&file);
		doc.save(out, 4);
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

		painter.fillRect(0, 0, width(), height(), QColor(255, 255, 255));

		// render unselected layers as background
		for (int l = 0; l < layer_id; ++l) {
			if (l == layer_id) continue;
			for (int i = 0; i < layers[l].shapes.size(); ++i) {
				layers[l].shapes[i]->draw(painter);
			}
		}

		painter.setPen(QColor(255, 255, 255, 160));
		painter.setBrush(QColor(255, 255, 255, 160));
		painter.drawRect(0, 0, width(), height());

		// render selected layer
		for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
			layers[layer_id].shapes[i]->draw(painter);
		}

		// render currently drawing shape
		if (mode == MODE_RECTANGLE || mode == MODE_POLYGON) {
			if (current_shape) {
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
			for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
				if (glm::length(layers[layer_id].shapes[i]->getRotationMarkerPosition() - layers[layer_id].shapes[i]->localCoordinate(glm::dvec2(e->x(), e->y()))) < 10) {
					mode = MODE_ROTATION;
					selected_shape = layers[layer_id].shapes[i];
					if (!layers[layer_id].shapes[i]->isSelected()) {
						unselectAll();
						layers[layer_id].shapes[i]->select();
					}
					update();
					return;
				}
			}

			// hit test for resize marker
			for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
				BoundingBox bbox = layers[layer_id].shapes[i]->boundingBox();
				if (glm::length(bbox.minPt - layers[layer_id].shapes[i]->localCoordinate(glm::dvec2(e->x(), e->y()))) < 10) {
					mode = MODE_RESIZE_TOP_LEFT;
					selected_shape = layers[layer_id].shapes[i];
					if (!layers[layer_id].shapes[i]->isSelected()) {
						unselectAll();
						layers[layer_id].shapes[i]->select();
					}
					update();
					return;
				}

				if (glm::length(bbox.maxPt - layers[layer_id].shapes[i]->localCoordinate(glm::dvec2(e->x(), e->y()))) < 10) {
					mode = MODE_RESIZE_BOTTOM_RIGHT;
					selected_shape = layers[layer_id].shapes[i];
					if (!layers[layer_id].shapes[i]->isSelected()) {
						unselectAll();
						layers[layer_id].shapes[i]->select();
					}
					update();
					return;
				}
			}

			// hit test for the shape
			for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
				if (layers[layer_id].shapes[i]->hit(glm::dvec2(e->x(), e->y()))) {
					if (!layers[layer_id].shapes[i]->isSelected()) {
						if (!ctrlPressed) {
							// If ctrl is not pressed, then deselect all other shapes.
							unselectAll();
						}
						layers[layer_id].shapes[i]->select();
					}
					update();
					return;
				}
			}

			unselectAll();
		}
		else if (mode == MODE_RECTANGLE) {
			if (current_shape) {
				// do nothing
			}
			else {
				// start drawing a rectangle
				unselectAll();
				current_shape = boost::shared_ptr<Shape>(new Rectangle(glm::dvec2(e->x(), e->y())));
				current_shape->startDrawing();
				setMouseTracking(true);
			}
		}
		else if (mode == MODE_POLYGON) {
			if (current_shape) {
				current_shape->addPoint(current_shape->localCoordinate(glm::dvec2(e->x(), e->y())));
			}
			else {
				// start drawing a polygon
				unselectAll();
				current_shape = boost::shared_ptr<Shape>(new Polygon(glm::dvec2(e->x(), e->y())));
				current_shape->startDrawing();
				setMouseTracking(true);
			}
		}

		update();
	}

	void Canvas::mouseMoveEvent(QMouseEvent* e) {
		if (mode == MODE_MOVE) {
			glm::dvec2 dir = glm::dvec2(e->x(), e->y()) - prev_mouse_pt;
			for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
				if (layers[layer_id].shapes[i]->isSelected()) {
					layers[layer_id].shapes[i]->translate(dir);
				}
			}
			prev_mouse_pt = glm::dvec2(e->x(), e->y());
			update();
		}
		else if (mode == MODE_ROTATION) {
			glm::dvec2 dir1 = prev_mouse_pt - selected_shape->worldCoordinate(selected_shape->getCenter());
			glm::dvec2 dir2 = glm::dvec2(e->x(), e->y()) - selected_shape->worldCoordinate(selected_shape->getCenter());
			double theta = atan2(dir2.y, dir2.x) - atan2(dir1.y, dir1.x);
			for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
				if (layers[layer_id].shapes[i]->isSelected()) {
					layers[layer_id].shapes[i]->rotate(theta);
				}
			}
			prev_mouse_pt = glm::dvec2(e->x(), e->y());
			update();
		}
		else if (mode == MODE_RESIZE_TOP_LEFT) {
			glm::dvec2 dir1 = selected_shape->boundingBox().minPt - selected_shape->boundingBox().maxPt;
			glm::dvec2 dir2 = selected_shape->localCoordinate(glm::dvec2(e->x(), e->y())) - selected_shape->boundingBox().maxPt;
			glm::dvec2 scale(dir2.x / dir1.x, dir2.y / dir1.y);
			for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
				if (layers[layer_id].shapes[i]->isSelected()) {
					layers[layer_id].shapes[i]->resize(scale, Shape::RESIZE_TOP_LEFT);
				}
			}
			prev_mouse_pt = glm::dvec2(e->x(), e->y());
			update();
		}
		else if (mode == MODE_RESIZE_BOTTOM_RIGHT) {
			glm::dvec2 dir1 = selected_shape->boundingBox().maxPt - selected_shape->boundingBox().minPt;
			glm::dvec2 dir2 = selected_shape->localCoordinate(glm::dvec2(e->x(), e->y())) - selected_shape->boundingBox().minPt;
			glm::dvec2 scale(dir2.x / dir1.x, dir2.y / dir1.y);
			for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
				if (layers[layer_id].shapes[i]->isSelected()) {
					layers[layer_id].shapes[i]->resize(scale, Shape::RESIZE_BOTTOM_RIGHT);
				}
			}
			prev_mouse_pt = glm::dvec2(e->x(), e->y());
			update();
		}
		else if (mode == MODE_RECTANGLE || mode == MODE_POLYGON) {
			if (current_shape) {
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
		if (mode == MODE_RECTANGLE || mode == MODE_POLYGON) {
			if (current_shape) {
				// The shape is created.
				current_shape->completeDrawing();
				current_shape->select();
				layers[layer_id].shapes.push_back(current_shape);
				mode = MODE_MOVE;
				current_shape.reset();
				mainWin->ui.actionMove->setChecked(true);
			}
		}

		setMouseTracking(false);

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
				current_shape.reset();
				setMouseTracking(false);
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