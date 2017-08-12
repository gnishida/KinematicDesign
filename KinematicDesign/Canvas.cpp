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
#include <QDate>
#include <glm/gtx/string_cast.hpp>
#include "MainWindow.h"
#include "Rectangle.h"
#include "Circle.h"
#include "Polygon.h"

namespace canvas {

	Canvas::Canvas(MainWindow* mainWin) : QWidget((QWidget*)mainWin) {
		this->mainWin = mainWin;
		ctrlPressed = false;
		shiftPressed = false;

		origin = QPoint(width() * 0.5, height() * 0.5);
		scale = 10.0;

		mode = MODE_SELECT;
		layers.resize(2);
		layer_id = 0;
		operation.reset();
		current_shape.reset();

		animation_timer = NULL;
		collision_check = true;

		selectedJoint = std::make_pair(-1, -1);
		linkage_type = LINKAGE_4R;
		linkage_subtype = -1;
		orderDefect = false;
		branchDefect = false;
		circuitDefect = false;
	}

	Canvas::~Canvas() {
	}

	void Canvas::clear() {
		for (int i = 0; i < layers.size(); ++i) {
			layers[i].clear();
		}
		selected_shape.reset();

		// select 1st layer
		setLayer(0);

		// clear the kinematic data
		kinematics.clear();
		solutions.clear();
		selectedJoint = std::make_pair(-1, -1);

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
		for (int i = layers[layer_id].shapes.size() - 1; i >= 0; --i) {
			if (layers[layer_id].shapes[i]->isSelected()) {
				for (int l = 0; l < layers.size(); l++) {
					layers[l].shapes.erase(layers[l].shapes.begin() + i);
				}
			}
		}

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

	/**
	 *
	 */
	void Canvas::circularRepeat(int num_repeat) {
		int N = layers[0].shapes.size();
		for (int i = 0; i < N; i++) {
			if (layers[0].shapes[i]->getSubType() == Shape::TYPE_BODY) {
				glm::dmat3x3 mat0 = layers[0].shapes[i]->getModelMatrix();

				bool moved = false;
				for (int j = 0; j < layers.size(); j++) {
					glm::dmat3x3 mat = layers[j].shapes[i]->getModelMatrix();
					if (mat != mat0) {
						moved = true;
						break;
					}
				}

				if (!moved) continue;
			}

			if (layers[0].shapes[i]->getSubType() == Shape::TYPE_BODY) {
				for (int j = 0; j < layers.size(); j++) {
					for (int k = 1; k < num_repeat; k++) {
						double theta = (double)k / num_repeat * 2 * 3.1415926535;

						// copy the shape
						boost::shared_ptr<Shape> shape = layers[j].shapes[i]->clone();

						// calculate the bounding box
						BoundingBox bbox = shape->boundingBox();

						// transform the shape
						glm::dvec2 offset = shape->worldCoordinate(bbox.center());
						shape->translate(-offset);
						shape->rotate(theta);
						shape->translate(glm::dvec2(offset.x * cos(theta) - offset.y * sin(theta), offset.x * sin(theta) + offset.y * cos(theta)));

						layers[j].shapes.push_back(shape);
					}
				}
			}
			else if (layers[0].shapes[i]->getSubType() == Shape::TYPE_LINKAGE_REGION) {
				for (int k = 1; k < num_repeat; k++) {
					double theta = (double)k / num_repeat * 2 * 3.1415926535;

					// copy the shape
					boost::shared_ptr<Shape> shape = layers[0].shapes[i]->clone();

					// calculate the bounding box
					BoundingBox bbox = shape->boundingBox();

					// transform the shape
					glm::dvec2 offset = shape->worldCoordinate(bbox.center());
					shape->translate(-offset);
					shape->rotate(theta);
					shape->translate(glm::dvec2(offset.x * cos(theta) - offset.y * sin(theta), offset.x * sin(theta) + offset.y * cos(theta)));

					layers[0].shapes.push_back(shape);
				}
			}
		}
	}

	void Canvas::setMode(int mode) {
		if (this->mode != mode) {
			this->mode = mode;

			// clear
			unselectAll();
			selectedJoint = std::make_pair(-1, -1);

			update();
		}
	}
	
	void Canvas::addLayer() {
		layers.push_back(layers.back().clone());
		setLayer(layers.size() - 1);
	}

	void Canvas::insertLayer() {
		layers.insert(layers.begin() + layer_id, layers[layer_id].clone());
		setLayer(layer_id);
	}

	void Canvas::deleteLayer() {
		// we assume that there must be at least two layers.
		if (layers.size() <= 2) return;

		layers.erase(layers.begin() + layer_id);
		if (layer_id >= layers.size()) {
			layer_id--;
		}
		setLayer(layer_id);
	}

	void Canvas::setLayer(int layer_id) {
		layers[this->layer_id].unselectAll();
		this->layer_id = layer_id;
		current_shape.reset();

		// change the mode to SELECT
		setMode(MODE_SELECT);

		update();
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

		// clear the kinematic data
		kinematics.clear();
		solutions.clear();

		// update the layer menu based on the loaded data
		mainWin->initLayerMenu(layers.size());

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

	void Canvas::saveKinematics(const QString& filename) {
		if (kinematics.size() > 0) kinematics[0].save(filename);
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
		for (int i = 0; i < kinematics.size(); i++) {
			kinematics[i].speedUp();
		}
	}

	void Canvas::speedDown() {
		for (int i = 0; i < kinematics.size(); i++) {
			kinematics[i].speedDown();
		}
	}

	void Canvas::invertSpeed() {
		for (int i = 0; i < kinematics.size(); i++) {
			kinematics[i].invertSpeed();
		}
	}

	void Canvas::stepForward() {
		if (animation_timer == NULL) {
			for (int i = 0; i < kinematics.size(); i++) {
				try {
					kinematics[i].stepForward(collision_check);
				}
				catch (char* ex) {
					kinematics[i].invertSpeed();
					std::cerr << "Animation is stopped by error:" << std::endl;
					std::cerr << ex << std::endl;
				}
			}
			update();
		}
	}

	void Canvas::stepBackward() {
		if (animation_timer == NULL) {
			for (int i = 0; i < kinematics.size(); i++) {
				try {
					kinematics[i].stepBackward(collision_check);
				}
				catch (char* ex) {
					kinematics[i].invertSpeed();
					std::cerr << "Animation is stopped by error:" << std::endl;
					std::cerr << ex << std::endl;
				}
			}
			update();
		}
	}

	void Canvas::showAssemblies(bool flag) {
		for (int i = 0; i < kinematics.size(); i++) {
			kinematics[i].showAssemblies(flag);
		}
		update();
	}

	void Canvas::showLinks(bool flag) {
		for (int i = 0; i < kinematics.size(); i++) {
			kinematics[i].showLinks(flag);
		}
		update();
	}

	void Canvas::showBodies(bool flag) {
		for (int i = 0; i < kinematics.size(); i++) {
			kinematics[i].showBodies(flag);
		}
		update();
	}

	glm::dvec2 Canvas::screenToWorldCoordinates(const glm::dvec2& p) {
		return screenToWorldCoordinates(p.x, p.y);
	}

	glm::dvec2 Canvas::screenToWorldCoordinates(double x, double y) {
		return glm::dvec2((x - origin.x()) / scale, -(y - origin.y()) / scale);
	}

	glm::dvec2 Canvas::worldToScreenCoordinates(const glm::dvec2& p) {
		return glm::dvec2(origin.x() + p.x * scale, origin.y() - p.y * scale);
	}

	void Canvas::calculateSolutions(int linkage_type, int num_samples, double sigma, bool avoid_branch_defect, bool rotatable_crank, double pose_error_weight, double trajectory_weight, double size_weight) {
		mainWin->ui.statusBar->showMessage("Please wait for a moment...");
		
		// change the mode to kinematics
		setMode(MODE_KINEMATICS);
		mainWin->ui.actionKinematics->setChecked(true);

		time_t start = clock();

		this->linkage_type = linkage_type;

		// get the geometry of fixed rigid bodies, moving bodies, linkage regions
		fixed_body_pts.clear();
		body_pts.clear();
		linkage_region_pts.clear();
		poses.clear();
		for (int i = 0; i < layers[0].shapes.size(); i++) {
			int subtype = layers[0].shapes[i]->getSubType();
			if (subtype == Shape::TYPE_BODY) {
				glm::dmat3x3 mat0 = layers[0].shapes[i]->getModelMatrix();

				bool moved = false;
				for (int j = 0; j < layers.size(); j++) {
					glm::dmat3x3 mat = layers[j].shapes[i]->getModelMatrix();
					if (mat != mat0) {
						moved = true;
						break;
					}
				}

				if (moved) {
					body_pts.push_back(layers[0].shapes[i]->getPoints());

					// calcualte poses of the moving body
					poses.resize(poses.size() + 1);
					for (int j = 0; j < layers.size(); j++) {
						poses.back().push_back(layers[j].shapes[i]->getModelMatrix());
					}
				}
				else {
					fixed_body_pts.push_back(layers[0].shapes[i]->getPoints());
				}
			}
			else if (subtype == Shape::TYPE_LINKAGE_REGION) {
				linkage_region_pts.push_back(layers[0].shapes[i]->getPoints());
			}
		}

		// if the linkage region is not specified, use a large enough region as default
		if (linkage_region_pts.size() < poses.size()) {
			int num_linkage_regions = linkage_region_pts.size();
			linkage_region_pts.resize(poses.size());
			for (int i = num_linkage_regions; i < poses.size(); i++) {
				linkage_region_pts[i].push_back(glm::dvec2(-40, -40));
				linkage_region_pts[i].push_back(glm::dvec2(-40, 40));
				linkage_region_pts[i].push_back(glm::dvec2(40, 40));
				linkage_region_pts[i].push_back(glm::dvec2(40, -40));
			}
		}

		kinematics.clear();

		solutions.resize(body_pts.size(), std::vector<kinematics::Solution>(2));
		for (int i = 0; i < body_pts.size(); i++) {
			time_t start = clock();

			boost::shared_ptr<kinematics::LinkageSynthesis> synthesis;
			if (linkage_type == LINKAGE_4R) {
				synthesis = boost::shared_ptr<kinematics::LinkageSynthesis>(new kinematics::LinkageSynthesis4R());
			}
			else if (linkage_type == LINKAGE_RRRP) {
				synthesis = boost::shared_ptr<kinematics::LinkageSynthesis>(new kinematics::LinkageSynthesisRRRP());
			}

			// calculate the circle point curve and center point curve
			synthesis->calculateSolution(poses[i], linkage_region_pts[i], num_samples, fixed_body_pts, body_pts[i], sigma, rotatable_crank, avoid_branch_defect, 1.0, solutions[i]);

			if (solutions[i].size() == 0) {
				mainWin->ui.statusBar->showMessage("No candidate was found.");
			}
			else if (solutions[i].size() == 0) {
				mainWin->ui.statusBar->showMessage("1 candidate was found.");
			}
			else {
				mainWin->ui.statusBar->showMessage(QString("%1 candidates were found.").arg(solutions[i].size()));
			}

			time_t end = clock();
			std::cout << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC << " sec for obtaining " << solutions[i].size() << " candidates." << std::endl;

			start = clock();
			if (linkage_type == LINKAGE_4R) {
				kinematics::Solution solution = synthesis->findBestSolution(poses[i], solutions[i], fixed_body_pts, body_pts[i], pose_error_weight, trajectory_weight, size_weight);

				// construct a linkage
				kinematics::Kinematics kin;
				kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(0, true, solution.fixed_point[0])));
				kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(1, true, solution.fixed_point[1])));
				kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(2, false, solution.moving_point[0])));
				kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(3, false, solution.moving_point[1])));
				kin.diagram.addLink(true, kin.diagram.joints[0], kin.diagram.joints[2]);
				kin.diagram.addLink(false, kin.diagram.joints[1], kin.diagram.joints[3]);
				kin.diagram.addLink(false, kin.diagram.joints[2], kin.diagram.joints[3]);

				// update the geometry
				kin.diagram.bodies.clear();
				kin.diagram.addBody(kin.diagram.joints[2], kin.diagram.joints[3], body_pts[i]);

				kinematics.push_back(kin);

				updateDefectFlag(solution.poses, kinematics[0]);
			}
			else if (linkage_type == LINKAGE_RRRP) {
				kinematics::Solution solution = synthesis->findBestSolution(poses[i], solutions[i], fixed_body_pts, body_pts[i], pose_error_weight, trajectory_weight, size_weight);

				// construct a linkage
				kinematics::Kinematics kin;
				kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(0, true, solution.fixed_point[0])));
				kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(1, true, solution.fixed_point[1])));
				kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(2, false, solution.moving_point[0])));
				kin.diagram.addJoint(boost::shared_ptr<kinematics::SliderHinge>(new kinematics::SliderHinge(3, false, solution.moving_point[1])));
				kin.diagram.addLink(true, kin.diagram.joints[0], kin.diagram.joints[2]);
				kin.diagram.addLink(false, kin.diagram.joints[1], kin.diagram.joints[3]);
				kin.diagram.addLink(false, kin.diagram.joints[2], kin.diagram.joints[3]);

				// update the geometry
				kin.diagram.bodies.clear();
				kin.diagram.addBody(kin.diagram.joints[2], kin.diagram.joints[3], body_pts[i]);

				kinematics.push_back(kin);

				updateDefectFlag(solution.poses, kinematics[0]);
			}

			end = clock();
			std::cout << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC << " sec for finding the best solution. " << std::endl;
		}

		// add the fixed rigid bodies to the fixed joints of all the linkages
		for (int i = 0; i < fixed_body_pts.size(); i++) {
			for (int j = 0; j < kinematics.size(); j++) {
				kinematics[j].diagram.addBody(kinematics[j].diagram.joints[0], kinematics[j].diagram.joints[1], fixed_body_pts[i]);
			}
		}

		// setup the kinematic system
		for (int i = 0; i < kinematics.size(); i++) {
			kinematics[i].diagram.initialize();
		}

		time_t end = clock();
		std::cout << "Total computation time was " << (double)(end - start) / CLOCKS_PER_SEC << " sec." << std::endl;

		update();

		//updateDefectFlag(poses[0], kinematics[0]);
	}

	/**
	 * Find the closest solution.
	 * 
	 * @param solutions	solution set
	 * @param pt		mouse position
	 * @param joint_id	0 -- driving crank / 1 -- follower
	 */
	int Canvas::findSolution(const std::vector<kinematics::Solution>& solutions, const glm::dvec2& pt, int joint_id) {
		int ans = -1;
		double min_dist = std::numeric_limits<double>::max();

		for (int i = 0; i < solutions.size(); i++) {
			double dist = glm::length(solutions[i].fixed_point[joint_id] - pt);
			if (dist < min_dist) {
				min_dist = dist;
				ans = i;
			}
		}

		return ans;
	}

	void Canvas::updateDefectFlag(const std::vector<glm::dmat3x3>& poses, const kinematics::Kinematics& kinematics) {
		int linkage_id = selectedJoint.first;

		boost::shared_ptr<kinematics::LinkageSynthesis> synthesis;
		if (linkage_type == LINKAGE_4R) {
			synthesis = boost::shared_ptr<kinematics::LinkageSynthesis>(new kinematics::LinkageSynthesis4R());
		}
		else if (linkage_type == LINKAGE_RRRP) {
			synthesis = boost::shared_ptr<kinematics::LinkageSynthesis>(new kinematics::LinkageSynthesisRRRP());
		}

		linkage_subtype = synthesis->getType(kinematics.diagram.joints[0]->pos, kinematics.diagram.joints[1]->pos, kinematics.diagram.joints[2]->pos, kinematics.diagram.joints[3]->pos);
		orderDefect = synthesis->checkOrderDefect(poses, kinematics.diagram.joints[0]->pos, kinematics.diagram.joints[1]->pos, kinematics.diagram.joints[2]->pos, kinematics.diagram.joints[3]->pos);
		branchDefect = synthesis->checkBranchDefect(poses, kinematics.diagram.joints[0]->pos, kinematics.diagram.joints[1]->pos, kinematics.diagram.joints[2]->pos, kinematics.diagram.joints[3]->pos);
		circuitDefect = synthesis->checkCircuitDefect(poses, kinematics.diagram.joints[0]->pos, kinematics.diagram.joints[1]->pos, kinematics.diagram.joints[2]->pos, kinematics.diagram.joints[3]->pos);
	}

	/*
	void Canvas::onDebug() {
		if (kinematics.size() >= 0) {
			boost::shared_ptr<kinematics::LinkageSynthesis> synthesis;
			if (linkage_type == LINKAGE_4R) {
				synthesis = boost::shared_ptr<kinematics::LinkageSynthesis>(new kinematics::LinkageSynthesis4R());
			}
			else if (linkage_type == LINKAGE_RRRP) {
				synthesis = boost::shared_ptr<kinematics::LinkageSynthesis>(new kinematics::LinkageSynthesisRRRP());
			}

			synthesis->checkOrderDefect(poses[0], kinematics[0].diagram.joints[0]->pos, kinematics[0].diagram.joints[1]->pos, kinematics[0].diagram.joints[2]->pos, kinematics[0].diagram.joints[3]->pos, true);
			synthesis->checkBranchDefect(poses[0], kinematics[0].diagram.joints[0]->pos, kinematics[0].diagram.joints[1]->pos, kinematics[0].diagram.joints[2]->pos, kinematics[0].diagram.joints[3]->pos, true);
			synthesis->checkCircuitDefect(poses[0], kinematics[0].diagram.joints[0]->pos, kinematics[0].diagram.joints[1]->pos, kinematics[0].diagram.joints[2]->pos, kinematics[0].diagram.joints[3]->pos, true);
		}
	}
	*/

	void Canvas::animation_update() {
		for (int i = 0; i < kinematics.size(); i++) {
			try {
				kinematics[i].stepForward(collision_check);
			}
			catch (char* ex) {
				kinematics[i].invertSpeed();
				//stop();
				std::cerr << "Animation is stopped by error:" << std::endl;
				std::cerr << ex << std::endl;
			}
		}

		update();

	}

	void Canvas::paintEvent(QPaintEvent *e) {
		QPainter painter(this);
		painter.fillRect(0, 0, width(), height(), QColor(255, 255, 255));

		// draw grid
		painter.save();
		painter.setPen(QPen(QColor(224, 224, 224), 1));
		for (int i = -2000; i <= 2000; i++) {
			painter.drawLine(origin.x() + i * 5 * scale, -10000 * scale + origin.y(), origin.x() + i * 5 * scale, 10000 * scale + origin.y());
			painter.drawLine(-10000 * scale + origin.x(), origin.y() + i * 5 * scale, 10000 * scale + origin.x(), origin.y() + i * 5 * scale);
		}
		painter.restore();

		if (mode != MODE_KINEMATICS) {
			// render unselected layers as background
			for (int l = 0; l < layers.size(); ++l) {
				if (l == layer_id) continue;
				for (int i = 0; i < layers[l].shapes.size(); ++i) {
					if (layers[l].shapes[i]->getSubType() == Shape::TYPE_BODY) {
						layers[l].shapes[i]->draw(painter, origin, scale);
					}
				}
			}

			// make the unselected layers faded
			painter.setPen(QColor(255, 255, 255, 160));
			painter.setBrush(QColor(255, 255, 255, 160));
			painter.drawRect(0, 0, width(), height());

			// render selected layer
			for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
				layers[layer_id].shapes[i]->draw(painter, origin, scale);
			}

			// render currently drawing shape
			if (mode == MODE_RECTANGLE || mode == MODE_CIRCLE || mode == MODE_POLYGON || mode == MODE_LINKAGE_REGION) {
				if (current_shape) {
					current_shape->draw(painter, origin, scale);
				}
			}
		}
		else {
			// draw solutions
			if (selectedJoint.first >= 0) {
				int linkage_id = selectedJoint.first;
				painter.setPen(QPen(QColor(255, 128, 128, 64), 1));
				painter.setBrush(QBrush(QColor(255, 128, 128, 64)));
				for (int i = 0; i < solutions[linkage_id].size(); i++) {
					painter.drawEllipse(origin.x() + solutions[linkage_id][i].fixed_point[0].x * scale, origin.y() - solutions[linkage_id][i].fixed_point[0].y * scale, 3, 3);
				}
				painter.setPen(QPen(QColor(128, 128, 255, 64), 1));
				painter.setBrush(QBrush(QColor(128, 128, 255, 64)));
				for (int i = 0; i < solutions[linkage_id].size(); i++) {
					painter.drawEllipse(origin.x() + solutions[linkage_id][i].fixed_point[1].x * scale, origin.y() - solutions[linkage_id][i].fixed_point[1].y * scale, 3, 3);
				}
			}

			// draw input shapes
			painter.setPen(QPen(QColor(0, 0, 0), 1, Qt::DashLine));
			painter.setBrush(QBrush(QColor(0, 0, 0, 0)));
			for (int i = 0; i < layers.size(); i++) {
				for (int j = 0; j < layers[i].shapes.size(); j++) {
					if (layers[i].shapes[j]->getSubType() == Shape::TYPE_BODY) {
						QPolygonF pts;
						std::vector<glm::dvec2>& body = layers[i].shapes[j]->getPoints();
						for (int k = 0; k < body.size(); k++) {
							pts.push_back(QPointF(origin.x() + body[k].x * scale, origin.y() - body[k].y * scale));
						}
						painter.drawPolygon(pts);
					}
				}
			}

			for (int i = 0; i < kinematics.size(); i++) {
				kinematics[i].draw(painter, origin, scale);
			}

			painter.setPen(QPen(QColor(0, 0, 0)));
			if (linkage_type == LINKAGE_4R) {
				if (linkage_subtype == 0) {
					painter.drawText(QPoint(6, 20), "Grashof (Drag-link)");
				}
				else if (linkage_subtype == 1) {
					painter.drawText(QPoint(6, 20), "Grashof (Crank-rocker)");
				}
				else if (linkage_subtype == 2) {
					painter.drawText(QPoint(6, 20), "Grashof (Rocker-crank)");
				}
				else if (linkage_subtype == 3) {
					painter.drawText(QPoint(6, 20), "Grashof (Double-rocker)");
				}
				else if (linkage_subtype == 4) {
					painter.drawText(QPoint(6, 20), "Non-Grashof (0-0 rocker)");
				}
				else if (linkage_subtype == 5) {
					painter.drawText(QPoint(6, 20), "Non-Grashof (pi-pi rocker)");
				}
				else if (linkage_subtype == 6) {
					painter.drawText(QPoint(6, 20), "Non-Grashof (pi-0 rocker)");
				}
				else if (linkage_subtype == 7) {
					painter.drawText(QPoint(6, 20), "Non-Grashof (0-pi rocker)");
				}
			}
			else if (linkage_type == LINKAGE_RRRP) {
				if (linkage_subtype == 0) {
					painter.drawText(QPoint(6, 20), "Rotatable crank");
				}
				else if (linkage_subtype == 1) {
					painter.drawText(QPoint(6, 20), "0-rocker");
				}
				else if (linkage_subtype == 2) {
					painter.drawText(QPoint(6, 20), "Pi-rocker");
				}
				else if (linkage_subtype == 3) {
					painter.drawText(QPoint(6, 20), "Rocker");
				}
			}
			painter.setPen(QPen(QColor(255, 0, 0)));
			if ((linkage_type == LINKAGE_4R && linkage_subtype >= 2) || (linkage_type == LINKAGE_RRRP && linkage_subtype >= 1)) {
				painter.drawText(QPoint(6, 36), "Grashof defect");
			}
			if (orderDefect) {
				painter.drawText(QPoint(6, 52), "Order defect");
			}
			if (branchDefect) {
				painter.drawText(QPoint(6, 68), "Branch defect");
			}
			if (circuitDefect) {
				painter.drawText(QPoint(6, 84), "Circuit defect");
			}
		}

		// draw axes
		painter.save();
		painter.setPen(QPen(QColor(128, 128, 128), 1));
		painter.drawLine(-10000 * scale + origin.x(), origin.y(), 10000 * scale + origin.x(), origin.y());
		painter.drawLine(origin.x(), -10000 * scale + origin.y(), origin.x(), 10000 * scale + origin.y());
		painter.restore();
	}

	void Canvas::mousePressEvent(QMouseEvent* e) {
		// This is necessary to get key event occured even after the user selects a menu.
		setFocus();

		if (e->buttons() & Qt::LeftButton) {
			if (mode == MODE_SELECT) {
				// hit test for rotation marker
				for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
					if (glm::length(layers[layer_id].shapes[i]->getRotationMarkerPosition(scale) - layers[layer_id].shapes[i]->localCoordinate(screenToWorldCoordinates(e->x(), e->y()))) < 10 / scale) {
						// start rotating
						mode = MODE_ROTATION;
						operation = boost::shared_ptr<Operation>(new RotateOperation(screenToWorldCoordinates(e->x(), e->y()), layers[layer_id].shapes[i]->worldCoordinate(layers[layer_id].shapes[i]->getCenter())));
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
					if (glm::length(bbox.minPt - layers[layer_id].shapes[i]->localCoordinate(screenToWorldCoordinates(e->x(), e->y()))) < 10 / scale) {
						// start resizing
						mode = MODE_RESIZE;
						operation = boost::shared_ptr<Operation>(new ResizeOperation(screenToWorldCoordinates(e->x(), e->y()), layers[layer_id].shapes[i]->worldCoordinate(bbox.maxPt)));
						selected_shape = layers[layer_id].shapes[i];
						if (!layers[layer_id].shapes[i]->isSelected()) {
							unselectAll();
							layers[layer_id].shapes[i]->select();
						}
						update();
						return;
					}

					if (glm::length(glm::dvec2(bbox.maxPt.x, bbox.minPt.y) - layers[layer_id].shapes[i]->localCoordinate(screenToWorldCoordinates(e->x(), e->y()))) < 10 / scale) {
						// start resizing
						mode = MODE_RESIZE;
						operation = boost::shared_ptr<Operation>(new ResizeOperation(screenToWorldCoordinates(e->x(), e->y()), layers[layer_id].shapes[i]->worldCoordinate(glm::dvec2(bbox.minPt.x, bbox.maxPt.y))));
						selected_shape = layers[layer_id].shapes[i];
						if (!layers[layer_id].shapes[i]->isSelected()) {
							unselectAll();
							layers[layer_id].shapes[i]->select();
						}
						update();
						return;
					}

					if (glm::length(glm::dvec2(bbox.minPt.x, bbox.maxPt.y) - layers[layer_id].shapes[i]->localCoordinate(screenToWorldCoordinates(e->x(), e->y()))) < 10 / scale) {
						// start resizing
						mode = MODE_RESIZE;
						operation = boost::shared_ptr<Operation>(new ResizeOperation(screenToWorldCoordinates(e->x(), e->y()), layers[layer_id].shapes[i]->worldCoordinate(glm::dvec2(bbox.maxPt.x, bbox.minPt.y))));
						selected_shape = layers[layer_id].shapes[i];
						if (!layers[layer_id].shapes[i]->isSelected()) {
							unselectAll();
							layers[layer_id].shapes[i]->select();
						}
						update();
						return;
					}

					if (glm::length(bbox.maxPt - layers[layer_id].shapes[i]->localCoordinate(screenToWorldCoordinates(e->x(), e->y()))) < 10 / scale) {
						// start resizing
						mode = MODE_RESIZE;
						operation = boost::shared_ptr<Operation>(new ResizeOperation(screenToWorldCoordinates(e->x(), e->y()), layers[layer_id].shapes[i]->worldCoordinate(bbox.minPt)));
						selected_shape = layers[layer_id].shapes[i];
						if (!layers[layer_id].shapes[i]->isSelected()) {
							unselectAll();
							layers[layer_id].shapes[i]->select();
						}
						update();
						return;
					}
				}

				// hit test for the selected shapes first
				for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
					if (layers[layer_id].shapes[i]->isSelected()) {
						if (layers[layer_id].shapes[i]->hit(screenToWorldCoordinates(e->x(), e->y()))) {
							// reselecting the already selected shapes
							mode = MODE_MOVE;
							operation = boost::shared_ptr<Operation>(new MoveOperation(screenToWorldCoordinates(e->x(), e->y())));
							update();
							return;
						}
					}
				}

				// hit test for the shape
				for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
					if (layers[layer_id].shapes[i]->getSubType() == Shape::TYPE_BODY) {
						if (layers[layer_id].shapes[i]->hit(screenToWorldCoordinates(e->x(), e->y()))) {
							// start moving
							mode = MODE_MOVE;
							operation = boost::shared_ptr<Operation>(new MoveOperation(screenToWorldCoordinates(e->x(), e->y())));
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
				}

				// hit test for the linkage region
				for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
					if (layers[layer_id].shapes[i]->getSubType() == Shape::TYPE_LINKAGE_REGION) {
						if (layers[layer_id].shapes[i]->hit(screenToWorldCoordinates(e->x(), e->y()))) {
							// start moving
							mode = MODE_MOVE;
							operation = boost::shared_ptr<Operation>(new MoveOperation(screenToWorldCoordinates(e->x(), e->y())));
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
					current_shape = boost::shared_ptr<Shape>(new Rectangle(Shape::TYPE_BODY, screenToWorldCoordinates(e->x(), e->y())));
					current_shape->startDrawing();
					setMouseTracking(true);
				}
			}
			else if (mode == MODE_CIRCLE) {
				if (current_shape) {
					// do nothing
				}
				else {
					// start drawing a circle
					unselectAll();
					current_shape = boost::shared_ptr<Shape>(new Circle(Shape::TYPE_BODY, screenToWorldCoordinates(e->x(), e->y())));
					current_shape->startDrawing();
					setMouseTracking(true);
				}
			}
			else if (mode == MODE_POLYGON) {
				if (current_shape) {
					current_shape->addPoint(current_shape->localCoordinate(screenToWorldCoordinates(e->x(), e->y())));
				}
				else {
					// start drawing a polygon
					unselectAll();
					current_shape = boost::shared_ptr<Shape>(new Polygon(Shape::TYPE_BODY, screenToWorldCoordinates(e->x(), e->y())));
					current_shape->startDrawing();
					setMouseTracking(true);
				}
			}
			else if (mode == MODE_LINKAGE_REGION) {
				if (current_shape) {
					current_shape->addPoint(current_shape->localCoordinate(screenToWorldCoordinates(e->x(), e->y())));
				}
				else {
					// start drawing a polygon
					unselectAll();
					current_shape = boost::shared_ptr<Shape>(new Polygon(Shape::TYPE_LINKAGE_REGION, screenToWorldCoordinates(e->x(), e->y())));
					current_shape->startDrawing();
					setMouseTracking(true);
				}
			}
			else if (mode == MODE_KINEMATICS) {
				// convert the mouse position to the world coordinate system
				glm::dvec2 pt((e->x() - origin.x()) / scale, -(e->y() - origin.y()) / scale);

				// select a joint to move
				selectedJoint = std::make_pair(-1, -1);
				double min_dist = 6;
				for (int i = 0; i < kinematics.size(); i++) {
					for (int j = 0; j < kinematics[i].diagram.joints.size(); j++) {
						if (j >= 2) continue;
						double dist = glm::length(kinematics[i].diagram.joints[j]->pos - pt);
						if (dist < min_dist) {
							min_dist = dist;
							selectedJoint = std::make_pair(i, j);
						}
					}
				}
			}
		}

		prev_mouse_pt = e->pos();
	}

	void Canvas::mouseMoveEvent(QMouseEvent* e) {
		if (e->buttons() & Qt::RightButton) {
			// move the camera
			if (e->buttons() & Qt::RightButton) {
				// translate the Origin
				origin += e->pos() - prev_mouse_pt;
				update();
			}
		}
		else {
			if (mode == MODE_MOVE) {
				boost::shared_ptr<MoveOperation> op = boost::static_pointer_cast<MoveOperation>(operation);
				glm::dvec2 dir = screenToWorldCoordinates(e->x(), e->y()) - op->pivot;
				for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
					if (layers[layer_id].shapes[i]->isSelected()) {
						if (layers[layer_id].shapes[i]->getSubType() == Shape::TYPE_BODY) {
							// if the shape is a regular body, move it only for the selected layer
							layers[layer_id].shapes[i]->translate(dir);
						}
						else {
							// if the shape is a linkage region, move it for all the layers in order to make the position of the linkage region the same across the layers
							for (int l = 0; l < layers.size(); l++) {
								layers[l].shapes[i]->translate(dir);
							}
						}
					}
				}
				op->pivot = screenToWorldCoordinates(e->x(), e->y());
				update();
			}
			else if (mode == MODE_ROTATION) {
				boost::shared_ptr<RotateOperation> op = boost::static_pointer_cast<RotateOperation>(operation);
				glm::dvec2 dir1 = op->pivot - op->rotation_center;
				glm::dvec2 dir2 = screenToWorldCoordinates(e->x(), e->y()) - op->rotation_center;
				double theta = atan2(dir2.y, dir2.x) - atan2(dir1.y, dir1.x);
				for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
					if (layers[layer_id].shapes[i]->isSelected()) {
						if (layers[layer_id].shapes[i]->getSubType() == Shape::TYPE_BODY) {
							// if the shape is a regular body, rotate it only for the selected layer
							layers[layer_id].shapes[i]->rotate(theta);
						}
						else {
							// if the shape is a linkage region, rotate it for all the layers in order to make the position of the linkage region the same across the layers
							for (int l = 0; l < layers.size(); l++) {
								layers[l].shapes[i]->rotate(theta);
							}
						}
					}
				}
				op->pivot = screenToWorldCoordinates(e->x(), e->y());
				update();
			}
			else if (mode == MODE_RESIZE) {
				boost::shared_ptr<ResizeOperation> op = boost::static_pointer_cast<ResizeOperation>(operation);
				glm::dvec2 resize_center = selected_shape->localCoordinate(op->resize_center);
				glm::dvec2 dir1 = selected_shape->localCoordinate(op->pivot) - resize_center;
				glm::dvec2 dir2 = selected_shape->localCoordinate(screenToWorldCoordinates(e->x(), e->y())) - resize_center;
				glm::dvec2 resize_scale(dir2.x / dir1.x, dir2.y / dir1.y);
				for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
					if (layers[layer_id].shapes[i]->isSelected()) {
						// resize the shape for all the layers in order to make the size of the shape the same across the layers
						for (int l = 0; l < layers.size(); l++) {
							layers[l].shapes[i]->resize(resize_scale, resize_center);
						}
					}
				}
				op->pivot = screenToWorldCoordinates(e->x(), e->y());
				update();
			}
			else if (mode == MODE_RECTANGLE || mode == MODE_CIRCLE || mode == MODE_POLYGON || mode == MODE_LINKAGE_REGION) {
				if (current_shape) {
					current_shape->updateByNewPoint(current_shape->localCoordinate(screenToWorldCoordinates(e->x(), e->y())), shiftPressed);
					update();
				}
			}
			else if (mode == MODE_KINEMATICS) {
				if (selectedJoint.first >= 0) {
					int linkage_id = selectedJoint.first;
					int joint_id = selectedJoint.second;
					glm::dvec2 pt = screenToWorldCoordinates(e->x(), e->y());

					if (ctrlPressed) {
						// move the selected joint
						kinematics[linkage_id].diagram.joints[joint_id]->pos = pt;

						updateDefectFlag(poses[linkage_id], kinematics[linkage_id]);
					}
					else {
						// select a solution
						glm::dvec2 pt = screenToWorldCoordinates(e->x(), e->y());
						int selectedSolution = findSolution(solutions[linkage_id], pt, joint_id);

						if (selectedSolution >= 0) {
							// move the selected joint (center point)
							kinematics[linkage_id].diagram.joints[joint_id]->pos = solutions[linkage_id][selectedSolution].fixed_point[joint_id];

							// move the other end joint (circle point)
							kinematics[linkage_id].diagram.joints[joint_id + 2]->pos = solutions[linkage_id][selectedSolution].moving_point[joint_id];

							// initialize the other link
							joint_id = 1 - joint_id;
							kinematics[linkage_id].diagram.joints[joint_id]->pos = solutions[linkage_id][selectedSolution].fixed_point[joint_id];
							kinematics[linkage_id].diagram.joints[joint_id + 2]->pos = solutions[linkage_id][selectedSolution].moving_point[joint_id];

							// initialize the other linkages
							for (int i = 0; i < kinematics.size(); i++) {
								if (i == linkage_id) continue;

								int selectedSolution = findSolution(solutions[i], kinematics[i].diagram.joints[0]->pos, 0);
								if (selectedSolution >= 0) {
									kinematics[i].diagram.joints[2]->pos = solutions[i][selectedSolution].moving_point[0];
									kinematics[i].diagram.joints[3]->pos = solutions[i][selectedSolution].moving_point[1];
								}
							}

							updateDefectFlag(solutions[linkage_id][selectedSolution].poses, kinematics[linkage_id]);
						}
					}

					// update the geometry
					for (int i = 0; i < body_pts.size(); i++) {
						kinematics[i].diagram.bodies.clear();
						kinematics[i].diagram.addBody(kinematics[i].diagram.joints[2], kinematics[i].diagram.joints[3], body_pts[i]);
					}
					for (int i = 0; i < fixed_body_pts.size(); i++) {
						for (int j = 0; j < kinematics.size(); j++) {
							kinematics[j].diagram.addBody(kinematics[j].diagram.joints[0], kinematics[j].diagram.joints[1], fixed_body_pts[i]);
						}
					}

					// setup the kinematic system
					for (int i = 0; i < kinematics.size(); i++) {
						kinematics[i].diagram.initialize();
					}
					update();

					//updateDefectFlag(poses[linkage_id], kinematics[linkage_id]);
				}
			}
		}
		
		prev_mouse_pt = e->pos();
	}

	void Canvas::mouseReleaseEvent(QMouseEvent* e) {
		if (mode == MODE_MOVE || mode == MODE_ROTATION || mode == MODE_RESIZE) {
			history.push(layers);
			mode = MODE_SELECT;
		}
	}

	void Canvas::mouseDoubleClickEvent(QMouseEvent* e) {
		if (mode == MODE_RECTANGLE || mode == MODE_CIRCLE || mode == MODE_POLYGON || mode == MODE_LINKAGE_REGION) {
			if (current_shape) {
				// The shape is created.
				current_shape->completeDrawing();
				//current_shape->select();
				for (int i = 0; i < layers.size(); i++) {
					layers[i].shapes.push_back(current_shape->clone());
				}
				layers[layer_id].shapes.back()->select();
				mode = MODE_SELECT;
				history.push(layers);
				current_shape.reset();
				operation.reset();
				mainWin->ui.actionSelect->setChecked(true);
			}
		}

		setMouseTracking(false);

		update();
	}

	void Canvas::wheelEvent(QWheelEvent* e) {
		double new_scale = scale + e->delta() * 0.01;
		new_scale = std::min(std::max(0.1, new_scale), 1000.0);

		// adjust the origin in order to keep the screen center
		glm::dvec2 p = screenToWorldCoordinates(width() * 0.5, height() * 0.5);
		origin.setX(width() * 0.5 - p.x * new_scale);
		origin.setY(height() * 0.5 + p.y * new_scale);
		
		scale = new_scale;

		update();
	}

	void Canvas::resizeEvent(QResizeEvent *e) {
		// adjust the origin in order to keep the screen center
		glm::dvec2 p = screenToWorldCoordinates(e->oldSize().width() * 0.5, e->oldSize().height() * 0.5);
		origin.setX(e->size().width() * 0.5 - p.x * scale);
		origin.setY(e->size().height() * 0.5 + p.y * scale);
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
			break;
		case Qt::Key_Space:
			// start/stop the animation
			if (animation_timer == NULL) {
				run();
			}
			else {
				stop();
			}
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
