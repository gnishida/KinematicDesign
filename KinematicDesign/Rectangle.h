#pragma once

#include "Shape.h"

namespace canvas {

	class Point;

	class Rectangle : public Shape {
	private:
		double width;
		double height;

	public:
		Rectangle();
		Rectangle(const glm::dvec2& point);
		Rectangle(QDomNode& node);
		~Rectangle();

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