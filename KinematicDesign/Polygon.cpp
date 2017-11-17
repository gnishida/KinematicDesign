#include "Polygon.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

namespace canvas {

	Polygon::Polygon() {
		theta = 0;
	}

	Polygon::Polygon(const glm::dvec2& point) {
		points.push_back(glm::dvec2());
		pos = point;
		theta = 0;

		current_point = glm::dvec2();
	}

	/**
	* Construct a polygon from the xml dom node.
	*/
	Polygon::Polygon(QDomNode& node) {
		QDomNode params_node = node.firstChild();
		while (!params_node.isNull()) {
			if (params_node.toElement().tagName() == "pose") {
				pos.x = params_node.toElement().attribute("x").toDouble();
				pos.y = params_node.toElement().attribute("y").toDouble();
				theta = params_node.toElement().attribute("theta").toDouble();
			}
			else if (params_node.toElement().tagName() == "point") {
				double x = params_node.toElement().attribute("x").toDouble();
				double y = params_node.toElement().attribute("y").toDouble();
				if (points.size() == 0 || points.back() != glm::dvec2(x, y)) {
					points.push_back(glm::dvec2(x, y));
				}
			}

			params_node = params_node.nextSibling();
		}

		// remove the last point if it is coincide with the first point
		if (points.size() >= 2 && points.front() == points.back()) {
			points.pop_back();
		}

		update3DGeometry();
	}

	Polygon::~Polygon() {
	}

	boost::shared_ptr<Shape> Polygon::clone() const {
		return boost::shared_ptr<Shape>(new Polygon(*this));
	}

	void Polygon::draw(QPainter& painter, const QColor& brush_color, const QPointF& origin, double scale) const {
		painter.save();

		painter.translate(origin.x() + pos.x * scale, origin.y() - pos.y * scale);
		painter.rotate(-theta / 3.14159265 * 180);

		if (selected || currently_drawing) {
			painter.setPen(QPen(QColor(0, 0, 255), 2));
		}
		else {
			painter.setPen(QPen(QColor(0, 0, 0), 1));
		}
		painter.setBrush(brush_color);

		// draw edges
		QPolygonF pol;
		for (int i = 0; i < points.size(); ++i) {
			pol.push_back(QPointF(points[i].x * scale, -points[i].y * scale));
		}
		if (currently_drawing) {
			pol.push_back(QPointF(current_point.x * scale, -current_point.y * scale));
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
			painter.drawRect(bbox.minPt.x * scale, -bbox.minPt.y * scale, bbox.width() * scale, -bbox.height() * scale);
			painter.setBrush(QBrush(QColor(255, 255, 255)));
			painter.drawRect(bbox.minPt.x * scale - 3, -bbox.minPt.y * scale - 3, 6, 6);
			painter.drawRect(bbox.maxPt.x * scale - 3, -bbox.minPt.y * scale - 3, 6, 6);
			painter.drawRect(bbox.maxPt.x * scale - 3, -bbox.maxPt.y * scale - 3, 6, 6);
			painter.drawRect(bbox.minPt.x * scale - 3, -bbox.maxPt.y * scale - 3, 6, 6);

			// show rotation marker
			painter.drawImage(bbox.center().x * scale - rotation_marker.width() / 2, -bbox.maxPt.y * scale - 10 - rotation_marker.height() / 2, rotation_marker);
		}

		painter.restore();
	}

	QDomElement Polygon::toXml(QDomDocument& doc, const QString& node_name) const {
		QDomElement shape_node = doc.createElement(node_name);
		shape_node.setAttribute("type", "polygon");

		QDomElement pose_node = doc.createElement("pose");
		pose_node.setAttribute("x", pos.x);
		pose_node.setAttribute("y", pos.y);
		pose_node.setAttribute("theta", theta);
		shape_node.appendChild(pose_node);

		for (int i = 0; i < points.size(); ++i) {
			QDomElement point_node = doc.createElement("point");
			point_node.setAttribute("x", points[i].x);
			point_node.setAttribute("y", points[i].y);
			shape_node.appendChild(point_node);
		}

		return shape_node;
	}

	void Polygon::addPoint(const glm::dvec2& point) {
		//points.push_back(point);
		if (points.size() == 0 || points.back() != current_point) {
			points.push_back(current_point);
		}
		current_point = point;
	}

	/**
	* Return the points of the rectangle in the world coordinate system.
	*/
	std::vector<glm::dvec2> Polygon::getPoints() const {
		std::vector<glm::dvec2> pts;
		for (int i = 0; i < points.size(); ++i) {
			pts.push_back(worldCoordinate(points[i]));
		}
		return pts;
	}

	void Polygon::updateByNewPoint(const glm::dvec2& point, bool shiftPressed) {
		current_point = point;
		if (shiftPressed) {
			if (abs(point.x - points.back().x) > abs(point.y - points.back().y)) {
				current_point.y = points.back().y;
			}
			else {
				current_point.x = points.back().x;
			}
		}
	}

	/**
	* Check if the point in the world coordinate is within the rectangle.
	*/
	bool Polygon::hit(const glm::dvec2& point) const {
		glm::dvec2 pt = localCoordinate(point);

		return withinPolygon(points, pt);
	}
	
	/**
	* Resize the polygon by the specified scale.
	* The resizing scale and the center of the resizing are specified as local coordinates.
	*/
	void Polygon::resize(const glm::dvec2& scale, const glm::dvec2& resize_center) {
		BoundingBox bbox = boundingBox();
		glm::dvec2 offset(resize_center.x * (1.0 - scale.x), resize_center.y * (1.0 - scale.y));
				
		for (int i = 0; i < points.size(); ++i) {
			points[i].x = (points[i].x - resize_center.x) * scale.x + resize_center.x - offset.x;
			points[i].y = (points[i].y - resize_center.y) * scale.y + resize_center.y - offset.y;
		}

		glm::dvec2 offset2(offset.x * cos(theta) - offset.y * sin(theta), offset.x * sin(theta) + offset.y * cos(theta));
		pos += offset2;

		update3DGeometry();
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

	bool Polygon::withinPolygon(const std::vector<glm::dvec2>& points, const glm::dvec2& pt) const {
		typedef boost::geometry::model::d2::point_xy<double> point_2d;

		boost::geometry::model::ring<point_2d> ring;
		for (int i = 0; i < points.size(); i++) {
			ring.push_back(point_2d(points[i].x, points[i].y));
		}

		boost::geometry::correct(ring);
		return boost::geometry::within(point_2d(pt.x, pt.y), ring);
	}

}