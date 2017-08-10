#include "Shape.h"
#include <QImage>
#include <kinematics.h>

namespace canvas {

	QImage Shape::rotation_marker = QImage("resources/rotation_marker.png").scaled(16, 16);
	std::vector<QBrush> Shape::brushes = { QBrush(QColor(0, 255, 0, 60)), QBrush(QColor(0, 0, 255, 30)) };

	Shape::Shape(int subtype) {
		this->subtype = subtype;
		selected = false;
		currently_drawing = false;
	}
	
	Shape::~Shape() {
	}

	/**
	* Return a model matrix which transform the local coordinates to the world coordinates.
	*/
	glm::dmat3x3 Shape::getModelMatrix() const {
		return glm::dmat3x3({ cos(theta), sin(theta), 0, -sin(theta), cos(theta), 0, pos.x, pos.y, 1 });
	}

	void Shape::select() {
		selected = true;
	}

	void Shape::unselect() {
		selected = false;
	}

	bool Shape::isSelected() const {
		return selected;
	}

	void Shape::startDrawing() {
		currently_drawing = true;
	}

	void Shape::completeDrawing() {
		currently_drawing = false;
	}

	void Shape::translate(const glm::dvec2& vec) {
		pos += vec;
	}

	void Shape::rotate(double angle) {
		glm::dvec2 c = boundingBox().center();
		glm::dvec2 c2(cos(theta) * c.x - sin(theta) * c.y, sin(theta) * c.x + cos(theta) * c.y);

		pos.x += c2.x * (1.0 - cos(angle)) + c2.y * sin(angle);
		pos.y += -c2.x * sin(angle) + c2.y * (1.0 - cos(angle));

		theta += angle;
	}

	glm::dvec2 Shape::getCenter() const {
		return boundingBox().center();
	}
	
	glm::dvec2 Shape::getRotationMarkerPosition(double scale) const {
		BoundingBox bbox = boundingBox();

		return glm::dvec2(bbox.center().x, bbox.maxPt.y + 10 / scale);
	}
	
	glm::dvec2 Shape::localCoordinate(const glm::dvec2& point) const {
		return glm::dvec2((point.x - pos.x) * cos(-theta) - (point.y - pos.y) * sin(-theta), (point.x - pos.x) * sin(-theta) + (point.y - pos.y) * cos(-theta));
	}

	glm::dvec2 Shape::worldCoordinate(const glm::dvec2& point) const {
		return glm::dvec2(point.x * cos(theta) - point.y * sin(theta) + pos.x, point.x * sin(theta) + point.y * cos(theta) + pos.y);
	}
}