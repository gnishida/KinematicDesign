#include "Design.h"
#include <QFile>
#include <QDate>
#include <QTextStream>
#include "Rectangle.h"
#include "Circle.h"
#include "Polygon.h"

namespace canvas {

	MovingBody MovingBody::clone() const {
		MovingBody ret;
		if (linkage_region) {
			ret.linkage_region = linkage_region->clone();
		}
		if (linkage_avoidance) {
			ret.linkage_avoidance = linkage_avoidance->clone();
		}

		ret.poses.resize(poses.size());
		for (int i = 0; i < poses.size(); i++) {
			ret.poses[i] = poses[i]->clone();
		}

		return ret;
	}

	void MovingBody::load(QDomNode& node) {
		QDomNode child_node = node.firstChild();
		while (!child_node.isNull()) {
			if (child_node.toElement().tagName() == "shape") {
				if (child_node.toElement().attribute("type") == "rectangle") {
					poses.push_back(boost::shared_ptr<Shape>(new Rectangle(child_node)));
				}
				else if (child_node.toElement().attribute("type") == "circle") {
					poses.push_back(boost::shared_ptr<Shape>(new Circle(child_node)));
				}
				else if (child_node.toElement().attribute("type") == "polygon") {
					poses.push_back(boost::shared_ptr<Shape>(new Polygon(child_node)));
				}
			}
			else if (child_node.toElement().tagName() == "linkage_region") {
				if (child_node.toElement().attribute("type") == "rectangle") {
					linkage_region = boost::shared_ptr<Shape>(new Rectangle(child_node));
				}
				else if (child_node.toElement().attribute("type") == "circle") {
					linkage_region = boost::shared_ptr<Shape>(new Circle(child_node));
				}
				else if (child_node.toElement().attribute("type") == "polygon") {
					linkage_region = boost::shared_ptr<Shape>(new Polygon(child_node));
				}
			}
			else if (child_node.toElement().tagName() == "linkage_avoidance") {
				if (child_node.toElement().attribute("type") == "rectangle") {
					linkage_avoidance = boost::shared_ptr<Shape>(new Rectangle(child_node));
				}
				else if (child_node.toElement().attribute("type") == "circle") {
					linkage_avoidance = boost::shared_ptr<Shape>(new Circle(child_node));
				}
				else if (child_node.toElement().attribute("type") == "polygon") {
					linkage_avoidance = boost::shared_ptr<Shape>(new Polygon(child_node));
				}
			}

			child_node = child_node.nextSibling();
		}

	}

	QDomElement MovingBody::toXml(QDomDocument& doc) {
		QDomElement moving_body_node = doc.createElement("moving_body");
		
		for (int i = 0; i < poses.size(); i++) {
			QDomElement shape_node = poses[i]->toXml(doc, "shape");
			moving_body_node.appendChild(shape_node);
		}

		if (linkage_region) {
			QDomElement linkage_region_node = linkage_region->toXml(doc, "linkage_region");
			moving_body_node.appendChild(linkage_region_node);
		}

		if (linkage_avoidance) {
			QDomElement linkage_avoidance_node = linkage_avoidance->toXml(doc, "linkage_avoidance");
			moving_body_node.appendChild(linkage_avoidance_node);
		}

		return moving_body_node;
	}

	Design::Design() {
		num_layers = 2;
		layer_id = 0;
	}

	Design Design::clone() const {
		Design ret;
		ret.num_layers = num_layers;
		ret.layer_id = layer_id;
		ret.fixed_bodies.resize(fixed_bodies.size());
		ret.moving_bodies.resize(moving_bodies.size());

		for (int i = 0; i < fixed_bodies.size(); i++) {
			ret.fixed_bodies[i] = fixed_bodies[i]->clone();
		}
		for (int i = 0; i < moving_bodies.size(); i++) {
			ret.moving_bodies[i] = moving_bodies[i].clone();
		}

		return ret;
	}

	/**
	 * Add a moving body.
	 */
	void Design::addMovingBody(boost::shared_ptr<Shape> shape) {
		moving_bodies.resize(moving_bodies.size() + 1);
		moving_bodies.back().poses.resize(num_layers);
		for (int i = 0; i < num_layers; i++) {
			moving_bodies.back().poses[i] = shape->clone();
		}
		moving_bodies.back().poses[layer_id]->select();
	}

	void Design::load(const QString& filename) {
		QFile file(filename);
		if (!file.open(QFile::ReadOnly | QFile::Text)) throw "File cannot open.";

		QDomDocument doc;
		doc.setContent(&file);

		QDomElement root = doc.documentElement();
		if (root.tagName() != "design")	throw "Invalid file format.";

		// clear the data
		clear();

		QDomNode node = root.firstChild();
		while (!node.isNull()) {
			if (node.toElement().tagName() == "fixed_body") {
				if (node.toElement().attribute("type") == "rectangle") {
					fixed_bodies.push_back(boost::shared_ptr<Shape>(new Rectangle(node)));
				}
				else if (node.toElement().attribute("type") == "circle") {
					fixed_bodies.push_back(boost::shared_ptr<Shape>(new Circle(node)));
				}
				else if (node.toElement().attribute("type") == "polygon") {
					fixed_bodies.push_back(boost::shared_ptr<Shape>(new Polygon(node)));
				}
			}
			else if (node.toElement().tagName() == "moving_body") {
				MovingBody moving_body;
				moving_body.load(node);
				moving_bodies.push_back(moving_body);
			}

			node = node.nextSibling();
		}

		// select 1st layer to display
		layer_id = 0;

		num_layers = 2;
		if (moving_bodies.size() > 0) {
			num_layers = moving_bodies[0].poses.size();
		}
	}

	void Design::save(const QString& filename) {
		QFile file(filename);
		if (!file.open(QFile::WriteOnly)) throw "File cannot open.";

		QDomDocument doc;

		// set root node
		QDomElement root = doc.createElement("design");
		root.setAttribute("author", "Gen Nishida");
		root.setAttribute("version", "1.0");
		root.setAttribute("date", QDate::currentDate().toString("MM/dd/yyyy"));
		doc.appendChild(root);

		// write fixed bodies
		for (int i = 0; i < fixed_bodies.size(); i++) {
			QDomElement fixed_body_node = fixed_bodies[i]->toXml(doc, "fixed_body");

			root.appendChild(fixed_body_node);
		}

		// write moving bodies
		for (int i = 0; i < moving_bodies.size(); i++) {
			QDomElement moving_body_node = moving_bodies[i].toXml(doc);
			
			root.appendChild(moving_body_node);
		}

		QTextStream out(&file);
		doc.save(out, 4);
	}

	void Design::clear() {
		fixed_bodies.clear();
		moving_bodies.clear();
	}

	/**
		* Select all the shapes of the current layer.
		*/
	void Design::selectAll() {
		for (int i = 0; i < fixed_bodies.size(); i++) {
			fixed_bodies[i]->select();
		}

		for (int i = 0; i < moving_bodies.size(); i++) {
			moving_bodies[i].poses[layer_id]->select();
			if (moving_bodies[i].linkage_region) {
				moving_bodies[i].linkage_region->select();
			}
			if (moving_bodies[i].linkage_avoidance) {
				moving_bodies[i].linkage_avoidance->select();
			}
		}
	}

	void Design::unselectAll() {
		for (int i = 0; i < fixed_bodies.size(); i++) {
			fixed_bodies[i]->unselect();
		}

		for (int i = 0; i < moving_bodies.size(); i++) {
			if (moving_bodies[i].linkage_region) {
				moving_bodies[i].linkage_region->unselect();
			}
			if (moving_bodies[i].linkage_avoidance) {
				moving_bodies[i].linkage_avoidance->unselect();
			}
			for (int k = 0; k < moving_bodies[i].poses.size(); k++) {
				moving_bodies[i].poses[k]->unselect();
			}
		}
	}

	void Design::deleteSelectedShapes() {
		for (int i = fixed_bodies.size() - 1; i >= 0; i--) {
			if (fixed_bodies[i]->isSelected()) fixed_bodies.erase(fixed_bodies.begin() + i);
		}

		for (int i = moving_bodies.size() - 1; i >= 0; i--) {
			if (moving_bodies[i].linkage_region && moving_bodies[i].linkage_region->isSelected()) moving_bodies[i].linkage_region.reset();
			if (moving_bodies[i].linkage_avoidance && moving_bodies[i].linkage_avoidance->isSelected()) moving_bodies[i].linkage_avoidance.reset();
			if (moving_bodies[i].poses[layer_id]->isSelected()) {
				moving_bodies.erase(moving_bodies.begin() + i);
			}
		}
	}

	void Design::copySelectedShapes() {
		copied_fixed_bodies.clear();
		for (int i = 0; i < fixed_bodies.size(); i++) {
			if (fixed_bodies[i]->isSelected()) {
				copied_fixed_bodies.push_back(fixed_bodies[i]->clone());
			}
		}
	}

	void Design::pasteCopiedShapes() {
		unselectAll();
		for (int i = 0; i < copied_fixed_bodies.size(); ++i) {
			boost::shared_ptr<Shape> shape = copied_fixed_bodies[i]->clone();
			shape->select();
			fixed_bodies.push_back(shape);
		}
	}

	/**
	 * Add a layer to the last.
	 * The previouly last shape will be copied to the new last layer.
	 */
	void Design::addLayer() {
		num_layers++;
		for (int i = 0; i < moving_bodies.size(); i++) {
			moving_bodies[i].poses.push_back(moving_bodies[i].poses.back()->clone());
		}
	}

	/**
	 * Insert a layer to right before the "layer_id"-th layer,
	 * and copy the shape in the previously "layer_id"-th layer to the new inserted layer.
	 */
	void Design::insertLayer() {
		num_layers++;
		for (int i = 0; i < moving_bodies.size(); i++) {
			moving_bodies[i].poses.insert(moving_bodies[i].poses.begin() + layer_id, moving_bodies[i].poses[layer_id]->clone());
		}
	}

	/**
	 * Delete the currently selected layer.
	 * If there is only two layers, return false without deleting a layer.
	 */
	bool Design::deleteLayer() {
		if (num_layers <= 2) return false;

		for (int i = 0; i < moving_bodies.size(); i++) {
			moving_bodies[i].poses.erase(moving_bodies[i].poses.begin() + layer_id);
		}
		num_layers--;
		if (layer_id >= num_layers) {
			layer_id--;
		}

		return true;
	}

	void Design::generate3DGeometry(RenderManager& renderManager) const {
		renderManager.removeObjects();
		for (int i = 0; i < fixed_bodies.size(); i++) {
			QString obj_name = QString("fixed_body_%1").arg(i);
			renderManager.addObject(obj_name, "", fixed_bodies[i]->getVertices(), true);
		}
		for (int i = 0; i < moving_bodies.size(); i++) {
			QString obj_name = QString("moving_body_%1").arg(i);
			renderManager.addObject(obj_name, "", moving_bodies[i].poses[layer_id]->getVertices(), true);
		}
	}

	bool Design::hitTest(const glm::dvec2& pt, bool multiple_selection, boost::shared_ptr<Shape>& selected_shape) {
		// hit test for the selected fixed bodies
		for (int i = 0; i < fixed_bodies.size(); i++) {
			if (fixed_bodies[i]->isSelected() && fixed_bodies[i]->hit(pt)) {
				selected_shape = fixed_bodies[i];
				return true;
			}
		}

		// hit test for the selected moving bodies
		for (int i = 0; i < moving_bodies.size(); i++) {
			if (moving_bodies[i].poses[layer_id]->isSelected() && moving_bodies[i].poses[layer_id]->hit(pt)) {
				selected_shape = moving_bodies[i].poses[layer_id];
				return true;
			}
		}

		// hit test for the selected linkage regions
		for (int i = 0; i < moving_bodies.size(); i++) {
			if (moving_bodies[i].linkage_region && moving_bodies[i].linkage_region->isSelected() && moving_bodies[i].linkage_region->hit(pt)) {
				selected_shape = moving_bodies[i].linkage_region;
				return true;
			}
		}

		// hit test for the selected linkage avoidance regions
		for (int i = 0; i < moving_bodies.size(); i++) {
			if (moving_bodies[i].linkage_avoidance && moving_bodies[i].linkage_avoidance->isSelected() && moving_bodies[i].linkage_avoidance->hit(pt)) {
				selected_shape = moving_bodies[i].linkage_avoidance;
				return true;
			}
		}

		// hit test for the non-selected fixed bodies
		for (int i = 0; i < fixed_bodies.size(); i++) {
			if (fixed_bodies[i]->hit(pt)) {
				if (!fixed_bodies[i]->isSelected()) {
					if (!multiple_selection) unselectAll();
					fixed_bodies[i]->select();
				}
				selected_shape = fixed_bodies[i];
				return true;
			}
		}

		// hit test for the non-selected moving bodies
		for (int i = 0; i < moving_bodies.size(); i++) {
			if (moving_bodies[i].poses[layer_id]->hit(pt)) {
				if (!moving_bodies[i].poses[layer_id]->isSelected()) {
					if (!multiple_selection) unselectAll();
					moving_bodies[i].poses[layer_id]->select();
				}
				selected_shape = moving_bodies[i].poses[layer_id];
				return true;
			}
		}

		// hit test for the non-selected linkage regions
		for (int i = 0; i < moving_bodies.size(); i++) {
			if (moving_bodies[i].linkage_region && moving_bodies[i].linkage_region->hit(pt)) {
				if (!moving_bodies[i].linkage_region->isSelected()) {
					if (!multiple_selection) unselectAll();
					moving_bodies[i].linkage_region->select();
				}
				selected_shape = moving_bodies[i].linkage_region;
				return true;
			}
		}

		// hit test for the non-selected linkage avoidance regions
		for (int i = 0; i < moving_bodies.size(); i++) {
			if (moving_bodies[i].linkage_avoidance && moving_bodies[i].linkage_avoidance->hit(pt)) {
				if (!moving_bodies[i].linkage_avoidance->isSelected()) {
					if (!multiple_selection) unselectAll();
					moving_bodies[i].linkage_avoidance->select();
				}
				selected_shape = moving_bodies[i].linkage_avoidance;
				return true;
			}
		}

		return false;
	}

	bool Design::hitTestRotationMarker(const glm::dvec2& pt, double scale, double threshold, boost::shared_ptr<Shape>& selected_shape, glm::dvec2& rotate_pivot) {
		for (int i = 0; i < fixed_bodies.size(); i++) {
			if (glm::length(fixed_bodies[i]->getRotationMarkerPosition(scale) - fixed_bodies[i]->localCoordinate(pt)) < threshold) {
				selected_shape = fixed_bodies[i];
				rotate_pivot = selected_shape->worldCoordinate(selected_shape->getCenter());
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
		}

		for (int i = 0; i < moving_bodies.size(); i++) {
			if (glm::length(moving_bodies[i].poses[layer_id]->getRotationMarkerPosition(scale) - moving_bodies[i].poses[layer_id]->localCoordinate(pt)) < threshold) {
				selected_shape = moving_bodies[i].poses[layer_id];
				rotate_pivot = selected_shape->worldCoordinate(selected_shape->getCenter());
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
		}

		for (int i = 0; i < moving_bodies.size(); i++) {
			if (moving_bodies[i].linkage_region && glm::length(moving_bodies[i].linkage_region->getRotationMarkerPosition(scale) - moving_bodies[i].linkage_region->localCoordinate(pt)) < threshold) {
				selected_shape = moving_bodies[i].linkage_region;
				rotate_pivot = selected_shape->worldCoordinate(selected_shape->getCenter());
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}

			if (moving_bodies[i].linkage_avoidance && glm::length(moving_bodies[i].linkage_avoidance->getRotationMarkerPosition(scale) - moving_bodies[i].linkage_avoidance->localCoordinate(pt)) < threshold) {
				selected_shape = moving_bodies[i].linkage_avoidance;
				rotate_pivot = selected_shape->worldCoordinate(selected_shape->getCenter());
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
		}

		return false;
	}

	bool Design::hitTestResizeMarker(const glm::dvec2& pt, double threshold, boost::shared_ptr<Shape>& selected_shape, glm::dvec2& resize_pivot) {
		for (int i = 0; i < fixed_bodies.size(); i++) {
			canvas::BoundingBox bbox = fixed_bodies[i]->boundingBox();

			if (glm::length(bbox.minPt - fixed_bodies[i]->localCoordinate(pt)) < threshold) {
				selected_shape = fixed_bodies[i];
				resize_pivot = selected_shape->worldCoordinate(bbox.maxPt);
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
			if (glm::length(glm::dvec2(bbox.maxPt.x, bbox.minPt.y) - fixed_bodies[i]->localCoordinate(pt)) < threshold) {
				selected_shape = fixed_bodies[i];
				resize_pivot = selected_shape->worldCoordinate(bbox.minPt.x, bbox.maxPt.y);
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
			if (glm::length(glm::dvec2(bbox.minPt.x, bbox.maxPt.y) - fixed_bodies[i]->localCoordinate(pt)) < threshold) {
				selected_shape = fixed_bodies[i];
				resize_pivot = selected_shape->worldCoordinate(bbox.maxPt.x, bbox.minPt.y);
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
			if (glm::length(bbox.maxPt - fixed_bodies[i]->localCoordinate(pt)) < threshold) {
				selected_shape = fixed_bodies[i];
				resize_pivot = selected_shape->worldCoordinate(bbox.minPt);
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
		}

		for (int i = 0; i < moving_bodies.size(); i++) {
			canvas::BoundingBox bbox = moving_bodies[i].poses[layer_id]->boundingBox();

			if (glm::length(bbox.minPt - moving_bodies[i].poses[layer_id]->localCoordinate(pt)) < threshold) {
				selected_shape = moving_bodies[i].poses[layer_id];
				resize_pivot = selected_shape->worldCoordinate(bbox.maxPt);
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
			if (glm::length(glm::dvec2(bbox.maxPt.x, bbox.minPt.y) - moving_bodies[i].poses[layer_id]->localCoordinate(pt)) < threshold) {
				selected_shape = moving_bodies[i].poses[layer_id];
				resize_pivot = selected_shape->worldCoordinate(bbox.minPt.x, bbox.maxPt.y);
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
			if (glm::length(glm::dvec2(bbox.minPt.x, bbox.maxPt.y) - moving_bodies[i].poses[layer_id]->localCoordinate(pt)) < threshold) {
				selected_shape = moving_bodies[i].poses[layer_id];
				resize_pivot = selected_shape->worldCoordinate(bbox.maxPt.x, bbox.minPt.y);
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
			if (glm::length(bbox.maxPt - moving_bodies[i].poses[layer_id]->localCoordinate(pt)) < threshold) {
				selected_shape = moving_bodies[i].poses[layer_id];
				resize_pivot = selected_shape->worldCoordinate(bbox.minPt);
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
		}

		for (int i = 0; i < moving_bodies.size(); i++) {
			if (!moving_bodies[i].linkage_region) continue;

			canvas::BoundingBox bbox = moving_bodies[i].linkage_region->boundingBox();

			if (glm::length(bbox.minPt - moving_bodies[i].linkage_region->localCoordinate(pt)) < threshold) {
				selected_shape = moving_bodies[i].linkage_region;
				resize_pivot = selected_shape->worldCoordinate(bbox.maxPt);
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
			if (glm::length(glm::dvec2(bbox.maxPt.x, bbox.minPt.y) - moving_bodies[i].linkage_region->localCoordinate(pt)) < threshold) {
				selected_shape = moving_bodies[i].linkage_region;
				resize_pivot = selected_shape->worldCoordinate(bbox.minPt.x, bbox.maxPt.y);
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
			if (glm::length(glm::dvec2(bbox.minPt.x, bbox.maxPt.y) - moving_bodies[i].linkage_region->localCoordinate(pt)) < threshold) {
				selected_shape = moving_bodies[i].linkage_region;
				resize_pivot = selected_shape->worldCoordinate(bbox.maxPt.x, bbox.minPt.y);
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
			if (glm::length(bbox.maxPt - moving_bodies[i].linkage_region->localCoordinate(pt)) < threshold) {
				selected_shape = moving_bodies[i].linkage_region;
				resize_pivot = selected_shape->worldCoordinate(bbox.minPt);
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
		}

		for (int i = 0; i < moving_bodies.size(); i++) {
			if (!moving_bodies[i].linkage_avoidance) continue;

			canvas::BoundingBox bbox = moving_bodies[i].linkage_avoidance->boundingBox();

			if (glm::length(bbox.minPt - moving_bodies[i].linkage_avoidance->localCoordinate(pt)) < threshold) {
				selected_shape = moving_bodies[i].linkage_avoidance;
				resize_pivot = selected_shape->worldCoordinate(bbox.maxPt);
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
			if (glm::length(glm::dvec2(bbox.maxPt.x, bbox.minPt.y) - moving_bodies[i].linkage_avoidance->localCoordinate(pt)) < threshold) {
				selected_shape = moving_bodies[i].linkage_avoidance;
				resize_pivot = selected_shape->worldCoordinate(bbox.minPt.x, bbox.maxPt.y);
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
			if (glm::length(glm::dvec2(bbox.minPt.x, bbox.maxPt.y) - moving_bodies[i].linkage_avoidance->localCoordinate(pt)) < threshold) {
				selected_shape = moving_bodies[i].linkage_avoidance;
				resize_pivot = selected_shape->worldCoordinate(bbox.maxPt.x, bbox.minPt.y);
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
			if (glm::length(bbox.maxPt - moving_bodies[i].linkage_avoidance->localCoordinate(pt)) < threshold) {
				selected_shape = moving_bodies[i].linkage_avoidance;
				resize_pivot = selected_shape->worldCoordinate(bbox.minPt);
				if (!selected_shape->isSelected()) {
					unselectAll();
					selected_shape->select();
				}
				return true;
			}
		}

		return false;
	}

	void Design::move(const glm::dvec2& dir, RenderManager& renderManager) {
		for (int i = 0; i < fixed_bodies.size(); i++) {
			if (fixed_bodies[i]->isSelected()) {
				fixed_bodies[i]->translate(dir);

				// update 3D geometry
				QString obj_name = QString("fixed_body_%1").arg(i);
				renderManager.removeObject(obj_name);
				renderManager.addObject(obj_name, "", fixed_bodies[i]->getVertices(), true);
			}
		}

		for (int i = 0; i < moving_bodies.size(); i++) {
			if (moving_bodies[i].poses[layer_id]->isSelected()) {
				moving_bodies[i].poses[layer_id]->translate(dir);

				// update 3D geometry
				QString obj_name = QString("moving_body_%1").arg(i);
				renderManager.removeObject(obj_name);
				renderManager.addObject(obj_name, "", moving_bodies[i].poses[layer_id]->getVertices(), true);
			}
		}

		for (int i = 0; i < moving_bodies.size(); i++) {
			if (moving_bodies[i].linkage_region && moving_bodies[i].linkage_region->isSelected()) {
				moving_bodies[i].linkage_region->translate(dir);
			}

			if (moving_bodies[i].linkage_avoidance && moving_bodies[i].linkage_avoidance->isSelected()) {
				moving_bodies[i].linkage_avoidance->translate(dir);
			}
		}
	}

	void Design::rotate(double theta, RenderManager& renderManager) {
		for (int i = 0; i < fixed_bodies.size(); i++) {
			if (fixed_bodies[i]->isSelected()) {
				fixed_bodies[i]->rotate(theta);

				// update 3D geometry
				QString obj_name = QString("fixed_body_%1").arg(i);
				renderManager.removeObject(obj_name);
				renderManager.addObject(obj_name, "", fixed_bodies[i]->getVertices(), true);
			}
		}

		for (int i = 0; i < moving_bodies.size(); i++) {
			if (moving_bodies[i].poses[layer_id]->isSelected()) {
				moving_bodies[i].poses[layer_id]->rotate(theta);

				// update 3D geometry
				QString obj_name = QString("moving_body_%1").arg(i);
				renderManager.removeObject(obj_name);
				renderManager.addObject(obj_name, "", moving_bodies[i].poses[layer_id]->getVertices(), true);
			}
		}

		for (int i = 0; i < moving_bodies.size(); i++) {
			if (moving_bodies[i].linkage_region && moving_bodies[i].linkage_region->isSelected()) {
				moving_bodies[i].linkage_region->rotate(theta);
			}

			if (moving_bodies[i].linkage_avoidance && moving_bodies[i].linkage_avoidance->isSelected()) {
				moving_bodies[i].linkage_avoidance->rotate(theta);
			}
		}
	}

	void Design::resize(const glm::dvec2& resize_scale, const glm::dvec2& resize_center, RenderManager& renderManager) {
		for (int i = 0; i < fixed_bodies.size(); i++) {
			if (fixed_bodies[i]->isSelected()) {
				fixed_bodies[i]->resize(resize_scale, resize_center);

				// update 3D geometry
				QString obj_name = QString("fixed_body_%1").arg(i);
				renderManager.removeObject(obj_name);
				renderManager.addObject(obj_name, "", fixed_bodies[i]->getVertices(), true);
			}
		}

		for (int i = 0; i < moving_bodies.size(); i++) {
			if (moving_bodies[i].poses[layer_id]->isSelected()) {
				// resize the shape for all the layers in order to make the size of the shape the same across the layers
				for (int k = 0; k < moving_bodies[i].poses.size(); k++) {
					moving_bodies[i].poses[k]->resize(resize_scale, resize_center);
				}

				// update 3D geometry
				QString obj_name = QString("moving_body_%1").arg(i);
				renderManager.removeObject(obj_name);
				renderManager.addObject(obj_name, "", moving_bodies[i].poses[layer_id]->getVertices(), true);
			}
		}

		for (int i = 0; i < moving_bodies.size(); i++) {
			if (moving_bodies[i].linkage_region && moving_bodies[i].linkage_region->isSelected()) {
				moving_bodies[i].linkage_region->resize(resize_scale, resize_center);
			}

			if (moving_bodies[i].linkage_avoidance && moving_bodies[i].linkage_avoidance->isSelected()) {
				moving_bodies[i].linkage_avoidance->resize(resize_scale, resize_center);
			}
		}
	}

}
