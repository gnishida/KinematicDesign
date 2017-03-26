#include "Rectangle.h"
#include "Point.h"
#include "Utils.h"

namespace canvas {

	Rectangle::Rectangle() : Shape() {
		width = 0;
		height = 0;
	}

	Rectangle::Rectangle(const glm::dvec2& point) : Shape() {
		model_mat = glm::translate(model_mat, glm::dvec3(point, 0));
		width = 0;
		height = 0;
	}

	Rectangle::Rectangle(QDomNode& node) : Shape() {
		QDomNode params_node = node.firstChild();
		while (!params_node.isNull()) {
			if (params_node.toElement().tagName() == "model_mat") {
				loadModelMat(params_node);
			}
			else if (params_node.toElement().tagName() == "params") {
				width = params_node.toElement().attribute("width").toDouble();
				height = params_node.toElement().attribute("height").toDouble();
			}

			params_node = params_node.nextSibling();
		}
	}

	Rectangle::~Rectangle() {
	}

	boost::shared_ptr<Shape> Rectangle::clone() const {
		return boost::shared_ptr<Shape>(new Rectangle(*this));
	}

	void Rectangle::draw(QPainter& painter) const {
		painter.save();

		painter.setTransform(getQTransform());

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
		pol.push_back(QPointF(0, 0));
		pol.push_back(QPointF(width, 0));
		pol.push_back(QPointF(width, height));
		pol.push_back(QPointF(0, height));
		painter.drawPolygon(pol);

		if (selected) {
			// show resize marker
			painter.setPen(QPen(QColor(0, 0, 0), 1));
			painter.setBrush(QBrush(QColor(255, 255, 255)));
			painter.drawRect(-3, -3, 6, 6);
			painter.drawRect(width - 3, -3, 6, 6);
			painter.drawRect(width - 3, height - 3, 6, 6);
			painter.drawRect(-3, height - 3, 6, 6);
			
			// show rotation marker
			painter.drawImage(width * 0.5 - rotation_marker.width() / 2, -10 - rotation_marker.height() / 2, rotation_marker);
		}

		painter.restore();
	}

	QDomElement Rectangle::toXml(QDomDocument& doc) const {
		QDomElement shape_node = doc.createElement("shape");
		shape_node.setAttribute("type", "rectangle");

		QDomElement model_mat_node = toModelMatXml(doc);
		shape_node.appendChild(model_mat_node);

		QDomElement params_node = doc.createElement("params");
		params_node.setAttribute("width", width);
		params_node.setAttribute("height", height);
		shape_node.appendChild(params_node);

		return shape_node;
	}

	void Rectangle::addPoint(const glm::dvec2& point) {
		// do nothing
	}

	void Rectangle::updateByNewPoint(const glm::dvec2& point) {
		width = point.x;
		height = point.y;
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
	
	
	void Rectangle::resize(const glm::dvec2& scale, const glm::dvec2& resize_center) {
		model_mat = glm::translate(model_mat, glm::dvec3(resize_center.x * (1.0 - scale.x), resize_center.y * (1.0 - scale.y), 0));

		width *= scale.x;
		height *= scale.y;
	}

	BoundingBox Rectangle::boundingBox() const {
		double min_x = std::min(0.0, width);
		double max_x = std::max(0.0, width);
		double min_y = std::min(0.0, height);
		double max_y = std::max(0.0, height);

		return BoundingBox(glm::dvec2(min_x, min_y), glm::dvec2(max_x, max_y));
	}

}