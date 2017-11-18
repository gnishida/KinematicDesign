#include "Shape.h"
#include <QImage>
#include <kinematics.h>

namespace canvas {

	QImage Shape::rotation_marker = QImage("resources/rotation_marker.png").scaled(16, 16);
	std::vector<QBrush> Shape::brushes = { QBrush(QColor(0, 255, 0, 60)), QBrush(QColor(0, 0, 255, 30)) };

	Shape::Shape() {
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

		update3DGeometry();
	}

	void Shape::translate(const glm::dvec2& vec) {
		pos += vec;

		update3DGeometry();
	}

	void Shape::rotate(double angle) {
		glm::dvec2 c = boundingBox().center();
		glm::dvec2 c2(cos(theta) * c.x - sin(theta) * c.y, sin(theta) * c.x + cos(theta) * c.y);

		pos.x += c2.x * (1.0 - cos(angle)) + c2.y * sin(angle);
		pos.y += -c2.x * sin(angle) + c2.y * (1.0 - cos(angle));

		theta += angle;

		update3DGeometry();
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

	glm::dvec2 Shape::worldCoordinate(double x, double y) const {
		return glm::dvec2(x * cos(theta) - y * sin(theta) + pos.x, x * sin(theta) + y * cos(theta) + pos.y);
	}

	void Shape::update3DGeometry() {
		vertices.clear();

		std::vector<glm::dvec2> points = getPoints();

		// if the last point is coincide witht the first point, remove the last point
		if (points.size() >= 2 && points.front() == points.back()) {
			points.pop_back();
		}

		std::vector<glm::vec2> pts(points.size());
		for (int i = 0; i < pts.size(); i++) {
			pts[i] = glm::vec2(points[i].x, points[i].y);
		}
		kinematics::glutils::drawPrism(pts, 10, glm::vec4(0.7, 1, 0.7, 1), glm::translate(glm::mat4(), glm::vec3(0, 0, -10)), vertices);
	}
}