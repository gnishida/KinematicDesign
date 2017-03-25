#include "Point.h"

namespace canvas {

	Point::Point() {
		x = 0;
		y = 0;
	}

	Point::Point(double x, double y, bool selected) {
		this->x = x;
		this->y = y;
	}

	Point::~Point() {
	}

	void Point::draw(QPainter& painter) {
		painter.save();

		painter.setPen(QPen(QColor(0, 0, 0), 1));
		painter.setBrush(QBrush(QColor(255, 0, 0)));
		painter.drawEllipse(QPointF(x, y), 5, 5);

		painter.restore();
	}

	void Point::addPoint(const glm::dvec2& point) {
		// do nothing
	}

	void Point::updateByNewPoint(const glm::dvec2& point) {
		x = point.x;
		y = point.y;
	}

	bool Point::hit(const glm::dvec2& point) {
		if (glm::length(glm::dvec2(x, y) - point) < 6) return true;
		else return false;
	}

	void Point::translate(const glm::dvec2& vec) {
		x += vec.x;
		y += vec.y;
	}

}