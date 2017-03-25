#pragma once

#include "Shape.h"

namespace canvas {
	class Point;

	class Polygon : public Shape {
	private:
		std::vector<glm::dvec2> points;
		glm::dvec2 current_point;

	public:
		Polygon();
		Polygon(const glm::dvec2& point);
		Polygon(QDomNode& node);
		~Polygon();

		boost::shared_ptr<Shape> clone();
		void draw(QPainter& painter) const;
		QDomElement toXml(QDomDocument& doc) const;
		void addPoint(const glm::dvec2& point);
		void updateByNewPoint(const glm::dvec2& point);
		bool hit(const glm::dvec2& point) const;
		void resize(const glm::dvec2& scale, int resize_type);
		BoundingBox boundingBox() const;
	};

}