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
#include "PinJoint.h"
#include "LinkageSolver.h"

namespace canvas {

	Canvas::Canvas(MainWindow* mainWin) : QWidget((QWidget*)mainWin) {
		this->mainWin = mainWin;
		ctrlPressed = false;
		shiftPressed = false;

		mode = MODE_SELECT;
		layers.resize(2);
		layer_id = 0;
		operation.reset();
		current_shape.reset();

		history.push(layers);

		animation_timer = NULL;
		simulation_speed = 0.01;
	}

	Canvas::~Canvas() {
	}

	void Canvas::clear() {
		for (int i = 0; i < layers.size(); ++i) {
			layers[i].clear();
		}
		selected_shape.reset();
		initial_diagrams.clear();
		kinematics.diagram.clear();
		update();
	}

	void Canvas::selectAll() {
		layers[layer_id].selectAll();
		mode = MODE_SELECT;
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

	void Canvas::undo() {
		try {
			layers = history.undo();
			update();
		}
		catch (char* ex) {
		}
	}

	void Canvas::redo() {
		try {
			layers = history.redo();
			update();
		}
		catch (char* ex) {
		}
	}

	void Canvas::copySelectedShapes() {
		layers[layer_id].copySelectedShapes(copied_shapes);
	}

	void Canvas::pasteCopiedShapes() {
		layers[layer_id].pasteCopiedShapes(copied_shapes);
		current_shape.reset();
		mode = MODE_SELECT;
		update();
	}

	void Canvas::setMode(int mode) {
		if (this->mode != mode) {
			this->mode = mode;
			unselectAll();
			update();
		}
	}

	void Canvas::setLayer(int layer_id) {
		if (this->layer_id != layer_id) {
			layers[this->layer_id].unselectAll();
			this->layer_id = layer_id;
			current_shape.reset();
			update();
		}
	}

	void Canvas::initialKinematicDiagram() {
		initial_diagrams = kinematics::LinkageSolver::initialKinematicDiagram(layers);
		update();
	}

	void Canvas::solveInverse() {
		try {
			//kinematics.diagram = kinematics::LinkageSolver::solve(initial_diagrams);
			kinematics.diagram = kinematics::LinkageSolver::optimize(initial_diagrams);
			initial_diagrams.clear();
			update();
		}
		catch (char* ex) {
			std::cout << ex << std::endl;
		}
	}

	void Canvas::open(const QString& filename) {
		QFile file(filename);
		if (!file.open(QFile::ReadOnly | QFile::Text)) throw "File cannot open.";

		QDomDocument doc;
		doc.setContent(&file);

		QDomElement root = doc.documentElement();
		if (root.tagName() != "design")	throw "Invalid file format.";

		// clear the data
		layers.clear();
		selected_shape.reset();
		initial_diagrams.clear();
		kinematics.diagram.clear();
		mode = MODE_SELECT;

		QDomNode layer_node = root.firstChild();
		while (!layer_node.isNull()) {
			if (layer_node.toElement().tagName() == "layer") {
				Layer layer;
				layer.load(layer_node.toElement());
				layers.push_back(layer);
			}

			layer_node = layer_node.nextSibling();
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
		for (int i = 0; i < layers.size(); ++i) {
			QDomElement layer_node = layers[i].toXml(doc);
			root.appendChild(layer_node);
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
		for (int l = 0; l < layers.size(); ++l) {
			if (l == layer_id) continue;
			for (int i = 0; i < layers[l].shapes.size(); ++i) {
				layers[l].shapes[i]->draw(painter);
			}
		}

		// draw diagrams
		painter.setPen(QPen(QColor(0, 0, 0, 255), 3));
		painter.setBrush(QColor(0, 0, 0, 255));
		for (int l = 0; l < initial_diagrams.size(); ++l) {
			if (l == layer_id) continue;
			for (int j = 0; j < initial_diagrams[l].links.size(); ++j) {
				QPolygonF polygon;
				for (int k = 0; k < initial_diagrams[l].links[j]->joints.size(); ++k) {
					polygon.push_back(QPointF(initial_diagrams[l].links[j]->joints[k]->pos.x, 800 - initial_diagrams[l].links[j]->joints[k]->pos.y));
					painter.drawEllipse(QPointF(initial_diagrams[l].links[j]->joints[k]->pos.x, 800 - initial_diagrams[l].links[j]->joints[k]->pos.y), 4, 4);
				}
				painter.drawPolygon(polygon);
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

		// draw initial diagrams
		if (initial_diagrams.size() > 0) {
			for (int j = 0; j < initial_diagrams[layer_id].links.size(); ++j) {
				QPolygonF polygon;
				for (int k = 0; k < initial_diagrams[layer_id].links[j]->joints.size(); ++k) {
					polygon.push_back(QPointF(initial_diagrams[layer_id].links[j]->joints[k]->pos.x, 800 - initial_diagrams[layer_id].links[j]->joints[k]->pos.y));
				}
				painter.setPen(QPen(QColor(0, 0, 0, 255), 3));
				painter.setBrush(QColor(0, 0, 0, 0));
				painter.drawPolygon(polygon);

				for (int k = 0; k < initial_diagrams[layer_id].links[j]->joints.size(); ++k) {
					if (initial_diagrams[layer_id].links[j]->joints[k]->ground) {
						painter.setPen(QPen(QColor(0, 0, 255, 255), 3));
						painter.setBrush(QColor(0, 0, 255, 255));
					}
					else {
						painter.setPen(QPen(QColor(0, 0, 0, 255), 3));
						painter.setBrush(QColor(0, 0, 0, 255));
					}
					painter.drawEllipse(QPointF(initial_diagrams[layer_id].links[j]->joints[k]->pos.x, 800 - initial_diagrams[layer_id].links[j]->joints[k]->pos.y), 4, 4);
				}
			}
		}

		kinematics.draw(painter);
	}

	void Canvas::mousePressEvent(QMouseEvent* e) {
		// This is necessary to get key event occured even after the user selects a menu.
		setFocus();

		if (mode == MODE_SELECT) {
			// hit test for rotation marker
			for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
				if (glm::length(layers[layer_id].shapes[i]->getRotationMarkerPosition() - layers[layer_id].shapes[i]->localCoordinate(glm::dvec2(e->x(), e->y()))) < 10) {
					// start rotating
					mode = MODE_ROTATION;
					operation = boost::shared_ptr<Operation>(new RotateOperation(glm::dvec2(e->x(), e->y()), layers[layer_id].shapes[i]->worldCoordinate(layers[layer_id].shapes[i]->getCenter())));
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
					// start resizing
					mode = MODE_RESIZE;
					operation = boost::shared_ptr<Operation>(new ResizeOperation(glm::dvec2(e->x(), e->y()), layers[layer_id].shapes[i]->worldCoordinate(bbox.maxPt)));
					selected_shape = layers[layer_id].shapes[i];
					if (!layers[layer_id].shapes[i]->isSelected()) {
						unselectAll();
						layers[layer_id].shapes[i]->select();
					}
					update();
					return;
				}

				if (glm::length(glm::dvec2(bbox.maxPt.x, bbox.minPt.y) - layers[layer_id].shapes[i]->localCoordinate(glm::dvec2(e->x(), e->y()))) < 10) {
					// start resizing
					mode = MODE_RESIZE;
					operation = boost::shared_ptr<Operation>(new ResizeOperation(glm::dvec2(e->x(), e->y()), layers[layer_id].shapes[i]->worldCoordinate(glm::dvec2(bbox.minPt.x, bbox.maxPt.y))));
					selected_shape = layers[layer_id].shapes[i];
					if (!layers[layer_id].shapes[i]->isSelected()) {
						unselectAll();
						layers[layer_id].shapes[i]->select();
					}
					update();
					return;
				}

				if (glm::length(glm::dvec2(bbox.minPt.x, bbox.maxPt.y) - layers[layer_id].shapes[i]->localCoordinate(glm::dvec2(e->x(), e->y()))) < 10) {
					// start resizing
					mode = MODE_RESIZE;
					operation = boost::shared_ptr<Operation>(new ResizeOperation(glm::dvec2(e->x(), e->y()), layers[layer_id].shapes[i]->worldCoordinate(glm::dvec2(bbox.maxPt.x, bbox.minPt.y))));
					selected_shape = layers[layer_id].shapes[i];
					if (!layers[layer_id].shapes[i]->isSelected()) {
						unselectAll();
						layers[layer_id].shapes[i]->select();
					}
					update();
					return;
				}

				if (glm::length(bbox.maxPt - layers[layer_id].shapes[i]->localCoordinate(glm::dvec2(e->x(), e->y()))) < 10) {
					// start resizing
					mode = MODE_RESIZE;
					operation = boost::shared_ptr<Operation>(new ResizeOperation(glm::dvec2(e->x(), e->y()), layers[layer_id].shapes[i]->worldCoordinate(bbox.minPt)));
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
					// start moving
					mode = MODE_MOVE;
					operation = boost::shared_ptr<Operation>(new MoveOperation(glm::dvec2(e->x(), e->y())));
					if (!layers[layer_id].shapes[i]->isSelected()) {
						if (!ctrlPressed) {
							// If CTRL is not pressed, then deselect all other shapes.
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
			boost::shared_ptr<MoveOperation> op = boost::static_pointer_cast<MoveOperation>(operation);
			glm::dvec2 dir = glm::dvec2(e->x(), e->y()) - op->pivot;
			for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
				if (layers[layer_id].shapes[i]->isSelected()) {
					layers[layer_id].shapes[i]->translate(dir);
				}
			}
			op->pivot = glm::dvec2(e->x(), e->y());
			update();
		}
		else if (mode == MODE_ROTATION) {
			boost::shared_ptr<RotateOperation> op = boost::static_pointer_cast<RotateOperation>(operation);
			glm::dvec2 dir1 = op->pivot - op->rotation_center;
			glm::dvec2 dir2 = glm::dvec2(e->x(), e->y()) - op->rotation_center;
			double theta = atan2(dir2.y, dir2.x) - atan2(dir1.y, dir1.x);
			for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
				if (layers[layer_id].shapes[i]->isSelected()) {
					layers[layer_id].shapes[i]->rotate(theta);
				}
			}
			op->pivot = glm::dvec2(e->x(), e->y());
			update();
		}
		else if (mode == MODE_RESIZE) {
			boost::shared_ptr<ResizeOperation> op = boost::static_pointer_cast<ResizeOperation>(operation);
			glm::dvec2 resize_center = selected_shape->localCoordinate(op->resize_center);
			glm::dvec2 dir1 = selected_shape->localCoordinate(op->pivot) - resize_center;
			glm::dvec2 dir2 = selected_shape->localCoordinate(glm::dvec2(e->x(), e->y())) - resize_center;
			glm::dvec2 scale(dir2.x / dir1.x, dir2.y / dir1.y);
			for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
				if (layers[layer_id].shapes[i]->isSelected()) {
					layers[layer_id].shapes[i]->resize(scale, resize_center);
				}
			}
			op->pivot = glm::dvec2(e->x(), e->y());
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
		if (mode == MODE_MOVE || mode == MODE_ROTATION || mode == MODE_RESIZE) {
			history.push(layers);
			mode = MODE_SELECT;
		}
	}

	void Canvas::mouseDoubleClickEvent(QMouseEvent* e) {
		if (mode == MODE_RECTANGLE || mode == MODE_POLYGON) {
			if (current_shape) {
				// The shape is created.
				current_shape->completeDrawing();
				current_shape->select();
				layers[layer_id].shapes.push_back(current_shape);
				mode = MODE_SELECT;
				history.push(layers);
				current_shape.reset();
				operation.reset();
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