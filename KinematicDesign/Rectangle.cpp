#include "Rectangle.h"

namespace canvas {

	Rectangle::Rectangle() {
		width = 0;
		height = 0;
		theta = 0;
	}

	Rectangle::Rectangle(const glm::dvec2& point) {
		width = 0;
		height = 0;
		pos = point;
		theta = 0;
	}

	/**
	 * Construct a rectangle from the xml dom node.
	 */
	Rectangle::Rectangle(QDomNode& node) {
		QDomNode params_node = node.firstChild();
		while (!params_node.isNull()) {
			if (params_node.toElement().tagName() == "pose") {
				pos.x = params_node.toElement().attribute("x").toDouble();
				pos.y = params_node.toElement().attribute("y").toDouble();
				theta = params_node.toElement().attribute("theta").toDouble();
			}
			else if (params_node.toElement().tagName() == "params") {
				width = params_node.toElement().attribute("width").toDouble();
				height = params_node.toElement().attribute("height").toDouble();
			}

			params_node = params_node.nextSibling();
		}

		update3DGeometry();
	}

	Rectangle::~Rectangle() {
	}

	boost::shared_ptr<Shape> Rectangle::clone() const {
		return boost::shared_ptr<Shape>(new Rectangle(*this));
	}

	void Rectangle::draw(QPainter& painter, const QColor& brush_color, const QPointF& origin, double scale) const {
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
		pol.push_back(QPointF(0, 0));
		pol.push_back(QPointF(width * scale, 0));
		pol.push_back(QPointF(width * scale, -height * scale));
		pol.push_back(QPointF(0, -height * scale));
		painter.drawPolygon(pol);

		if (selected) {
			// show resize marker
			painter.setPen(QPen(QColor(0, 0, 0), 1));
			painter.setBrush(QBrush(QColor(255, 255, 255)));
			painter.drawRect(-3, -3, 6, 6);
			painter.drawRect(width * scale - 3, -3, 6, 6);
			painter.drawRect(width * scale - 3, -height * scale - 3, 6, 6);
			painter.drawRect(-3, -height * scale - 3, 6, 6);
			
			BoundingBox bbox = boundingBox();

			// show rotation marker
			painter.drawImage(width * scale  * 0.5 - rotation_marker.width() / 2, -bbox.maxPt.y * scale - 10 - rotation_marker.height() / 2, rotation_marker);
		}

		painter.restore();
	}

	QDomElement Rectangle::toXml(QDomDocument& doc, const QString& node_name) const {
		QDomElement shape_node = doc.createElement(node_name);
		shape_node.setAttribute("type", "rectangle");

		QDomElement pose_node = doc.createElement("pose");
		pose_node.setAttribute("x", pos.x);
		pose_node.setAttribute("y", pos.y);
		pose_node.setAttribute("theta", theta);
		shape_node.appendChild(pose_node);
		
		QDomElement params_node = doc.createElement("params");
		params_node.setAttribute("width", width);
		params_node.setAttribute("height", height);
		shape_node.appendChild(params_node);

		return shape_node;
	}

	void Rectangle::addPoint(const glm::dvec2& point) {
		// do nothing
	}

	/**
	 * Return the points of the rectangle in the world coordinate system.
	 */
	std::vector<glm::dvec2> Rectangle::getPoints() const {
		std::vector<glm::dvec2> points;
		points.push_back(worldCoordinate(glm::dvec2(0, 0)));
		points.push_back(worldCoordinate(glm::dvec2(width, 0)));
		points.push_back(worldCoordinate(glm::dvec2(width, height)));
		points.push_back(worldCoordinate(glm::dvec2(0, height)));
		return points;
	}

	void Rectangle::updateByNewPoint(const glm::dvec2& point, bool shiftPressed) {
		width = point.x;
		height = point.y;
		if (shiftPressed) {
			width = std::max(width, height);
			if (height * width >= 0) height = width;
			else height = -width;
		}
	}

	/**
	 * Check if the point in the world coordinate is within the rectangle.
	 */
	bool Rectangle::hit(const glm::dvec2& point) const {
		glm::dvec2 pt = localCoordinate(point);

		double min_x = std::min(0.0, width);
		double max_x = std::max(0.0, width);
		double min_y = std::min(0.0, height);
		double max_y = std::max(0.0, height);

		if (pt.x < min_x || pt.x > max_x || pt.y < min_y || pt.y > max_y) return false;
		else return true;
	}
	
	/**
	 * Resize the rectangle by the specified scale.
	 * The resizing scale and the center of the resizing are specified as local coordinates.
	 */
	void Rectangle::resize(const glm::dvec2& scale, const glm::dvec2& resize_center) {
		glm::dvec2 dir(resize_center.x * (1.0 - scale.x), resize_center.y * (1.0 - scale.y));
		
		pos.x += dir.x * cos(theta) - dir.y * sin(theta);
		pos.y += dir.x * sin(theta) + dir.y * cos(theta);

		width *= scale.x;
		height *= scale.y;

		update3DGeometry();
	}

	BoundingBox Rectangle::boundingBox() const {
		double min_x = std::min(0.0, width);
		double max_x = std::max(0.0, width);
		double min_y = std::min(0.0, height);
		double max_y = std::max(0.0, height);

		return BoundingBox(glm::dvec2(min_x, min_y), glm::dvec2(max_x, max_y));
	}

}