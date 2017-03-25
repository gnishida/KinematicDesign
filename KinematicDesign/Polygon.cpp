#include "Polygon.h"
#include "Point.h"
#include "Utils.h"

namespace canvas {

	Polygon::Polygon() : Shape() {
	}

	Polygon::Polygon(const glm::dvec2& point) : Shape() {
		points.push_back(glm::dvec2());
		model_mat = glm::translate(model_mat, glm::dvec3(point, 0));
		transform.translate(point.x, point.y);

		current_point = glm::dvec2();
	}

	Polygon::Polygon(QDomNode& node) : Shape() {
		QDomNode params_node = node.firstChild();
		while (!params_node.isNull()) {
			if (params_node.toElement().tagName() == "model_mat") {
				loadModelMat(params_node);
			}
			else if (params_node.toElement().tagName() == "transform") {
				loadTransform(params_node);
			}
			else if (params_node.toElement().tagName() == "point") {
				double x = params_node.toElement().attribute("x").toDouble();
				double y = params_node.toElement().attribute("y").toDouble();

				points.push_back(glm::dvec2(x, y));
			}

			params_node = params_node.nextSibling();
		}
	}

	Polygon::~Polygon() {
	}

	boost::shared_ptr<Shape> Polygon::clone() {
		return boost::shared_ptr<Shape>(new Polygon(*this));
	}

	void Polygon::draw(QPainter& painter) const {
		painter.save();

		painter.setTransform(transform);

		if (selected || currently_drawing) {
			painter.setPen(QPen(QColor(0, 0, 255), 2));
		}
		else {
			painter.setPen(QPen(QColor(0, 0, 0), 1));
		}
		if (currently_drawing) {
			painter.setBrush(QBrush(QColor(0, 0, 0, 0)));
		}
		else {
			painter.setBrush(QBrush(QColor(0, 255, 0, 60)));
		}

		// draw edges
		QPolygonF pol;
		for (int i = 0; i < points.size(); ++i) {
			pol.push_back(QPointF(points[i].x, points[i].y));
		}
		if (currently_drawing) {
			pol.push_back(QPointF(current_point.x, current_point.y));
			painter.drawPolyline(pol);
		}
		else {
			painter.drawPolygon(pol);
		}

		if (selected) {
			// show resize marker
			BoundingBox bbox = boundingBox();
			painter.setPen(QPen(QColor(0, 0, 0), 1));
			painter.setBrush(QBrush(QColor(0, 0, 0, 0)));
			painter.drawRect(bbox.minPt.x, bbox.minPt.y, bbox.width(), bbox.height());
			painter.setBrush(QBrush(QColor(255, 255, 255)));
			painter.drawRect(bbox.minPt.x - 3, bbox.minPt.y - 3, 6, 6);
			painter.drawRect(bbox.maxPt.x - 3, bbox.minPt.y - 3, 6, 6);
			painter.drawRect(bbox.maxPt.x - 3, bbox.maxPt.y - 3, 6, 6);
			painter.drawRect(bbox.minPt.x - 3, bbox.maxPt.y - 3, 6, 6);

			// show rotation marker
			glm::dvec2 pos = getRotationMarkerPosition();
			painter.drawImage(pos.x - rotation_marker.width() / 2, pos.y - rotation_marker.height() / 2, rotation_marker);
		}

		painter.restore();
	}

	QDomElement Polygon::toXml(QDomDocument& doc) const {
		QDomElement shape_node = doc.createElement("shape");
		shape_node.setAttribute("type", "polygon");

		QDomElement model_mat_node = toModelMatXml(doc);
		shape_node.appendChild(model_mat_node);
		QDomElement transform_node = toTransformXml(doc);
		shape_node.appendChild(transform_node);

		for (int i = 0; i < points.size(); ++i) {
			QDomElement point_node = doc.createElement("point");
			point_node.setAttribute("x", points[i].x);
			point_node.setAttribute("y", points[i].y);
			shape_node.appendChild(point_node);
		}

		return shape_node;
	}

	void Polygon::addPoint(const glm::dvec2& point) {
		points.push_back(point);
		current_point = point;
	}

	void Polygon::updateByNewPoint(const glm::dvec2& point) {
		current_point = point;
	}

	/**
	* Check if the point in the world coordinate is within the rectangle.
	*/
	bool Polygon::hit(const glm::dvec2& point) const {
		glm::dvec2 pt = localCoordinate(point);

		return kinematics::pointWithinPolygon(pt, points);
	}
		
	void Polygon::resize(const glm::dvec2& scale, int resize_type) {
		BoundingBox bbox = boundingBox();

		if (resize_type == RESIZE_TOP_LEFT) {
			model_mat = glm::translate(model_mat, glm::dvec3(bbox.width() - bbox.width() * scale.x, bbox.height() - bbox.height() * scale.y, 0));
			transform.translate(bbox.width() - bbox.width() * scale.x, bbox.height() - bbox.height() * scale.y);
		}
		else if (resize_type == RESIZE_BOTTOM_RIGHT) {
			// do nothing
		}

		for (int i = 0; i < points.size(); ++i) {
			points[i].x = (points[i].x - bbox.minPt.x) * scale.x + bbox.minPt.x;
			points[i].y = (points[i].y - bbox.minPt.y) * scale.y + bbox.minPt.y;
		}
	}

	BoundingBox Polygon::boundingBox() const {
		double min_x = std::numeric_limits<double>::max();
		double max_x = -std::numeric_limits<double>::max();
		double min_y = std::numeric_limits<double>::max();
		double max_y = -std::numeric_limits<double>::max();
		for (int i = 0; i < points.size(); ++i) {
			min_x = std::min(min_x, points[i].x);
			max_x = std::max(max_x, points[i].x);
			min_y = std::min(min_y, points[i].y);
			max_y = std::max(max_y, points[i].y);
		}

		return BoundingBox(glm::dvec2(min_x, min_y), glm::dvec2(max_x, max_y));
	}

}