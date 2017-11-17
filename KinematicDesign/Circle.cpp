#include "Circle.h"

namespace canvas {

	Circle::Circle() {
		width = 0;
		height = 0;
	}

	Circle::Circle(const glm::dvec2& point) {
		width = 0;
		height = 0;
		pos = point;
		theta = 0;
	}

	/**
	* Construct a rectangle from the xml dom node.
	*/
	Circle::Circle(QDomNode& node) {
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

	Circle::~Circle() {
	}

	boost::shared_ptr<Shape> Circle::clone() const {
		return boost::shared_ptr<Shape>(new Circle(*this));
	}

	void Circle::draw(QPainter& painter, const QColor& brush_color, const QPointF& origin, double scale) const {
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

		// draw circle
		QPolygonF pol;
		for (int i = 0; i < 36; i++) {
			double theta = (double)i * 3.141592653 * 2 / 36;
			pol.push_back(QPointF((width * 0.5 + width * 0.5 * cos(theta)) * scale, (-height * 0.5 + height * 0.5 * sin(theta)) * scale));
		}
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

	QDomElement Circle::toXml(QDomDocument& doc, const QString& node_name) const {
		QDomElement shape_node = doc.createElement(node_name);
		shape_node.setAttribute("type", "circle");

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

	void Circle::addPoint(const glm::dvec2& point) {
		// do nothing
	}

	/**
	* Return the points of the rectangle in the world coordinate system.
	*/
	std::vector<glm::dvec2> Circle::getPoints() const {
		std::vector<glm::dvec2> points;
		for (int i = 0; i < 36; i++) {
			double theta = (double)i * 3.141592653 * 2 / 36;
			points.push_back(worldCoordinate(glm::dvec2(width * 0.5 + width * 0.5 * cos(theta), height * 0.5 + height * 0.5 * sin(theta))));
		}
		return points;
	}

	void Circle::updateByNewPoint(const glm::dvec2& point, bool shiftPressed) {
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
	bool Circle::hit(const glm::dvec2& point) const {
		glm::dvec2 pt = localCoordinate(point);

		glm::dvec2 v((pt.x - width * 0.5) / width * 2, (pt.y - height * 0.5) / height * 2);
		if (v.x * v.x + v.y * v.y <= 1) return true;
		else return false;
	}

	/**
	* Resize the rectangle by the specified scale.
	* The resizing scale and the center of the resizing are specified as local coordinates.
	*/
	void Circle::resize(const glm::dvec2& scale, const glm::dvec2& resize_center) {
		glm::dvec2 dir(resize_center.x * (1.0 - scale.x), resize_center.y * (1.0 - scale.y));

		pos.x += dir.x * cos(theta) - dir.y * sin(theta);
		pos.y += dir.x * sin(theta) + dir.y * cos(theta);

		width *= scale.x;
		height *= scale.y;

		update3DGeometry();
	}

	BoundingBox Circle::boundingBox() const {
		double min_x = std::min(0.0, width);
		double max_x = std::max(0.0, width);
		double min_y = std::min(0.0, height);
		double max_y = std::max(0.0, height);

		return BoundingBox(glm::dvec2(min_x, min_y), glm::dvec2(max_x, max_y));
	}

}