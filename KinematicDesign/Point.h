#pragma once

#include "Shape.h"

namespace canvas {

	class Point {
	public:
		double x;
		double y;

	public:
		Point();
		Point(double x, double y, bool selected);
		~Point();

		void draw(QPainter& painter);
		void addPoint(const glm::dvec2& point);
		void updateByNewPoint(const glm::dvec2& point);
		bool hit(const glm::dvec2& point);
		void translate(const glm::dvec2& vec);
	};

}