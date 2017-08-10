#pragma once

#include "Shape.h"

namespace canvas {

	class Point;

	class Rectangle : public Shape {
	private:
		double width;
		double height;

	public:
		Rectangle(int subtype);
		Rectangle(int subtype, const glm::dvec2& point);
		Rectangle(int subtype, QDomNode& node);
		~Rectangle();

		boost::shared_ptr<Shape> clone() const;
		void draw(QPainter& painter, const QPointF& origin, double scale) const;
		QDomElement toXml(QDomDocument& doc) const;
		void addPoint(const glm::dvec2& point);
		std::vector<glm::dvec2> getPoints() const;
		void updateByNewPoint(const glm::dvec2& point, bool shiftPressed);
		bool hit(const glm::dvec2& point) const;
		void resize(const glm::dvec2& scale, const glm::dvec2& resize_center);
		BoundingBox boundingBox() const;
	};

}