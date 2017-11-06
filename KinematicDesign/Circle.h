#pragma once

#include "Shape.h"

namespace canvas {

	class Circle : public Shape {
	private:
		double width;
		double height;


	public:
		Circle();
		Circle(const glm::dvec2& point);
		Circle(QDomNode& node);
		~Circle();

		boost::shared_ptr<Shape> clone() const;
		void draw(QPainter& painter, const QColor& brush_color, const QPointF& origin, double scale) const;
		QDomElement toXml(QDomDocument& doc, const QString& node_name) const;
		void addPoint(const glm::dvec2& point);
		std::vector<glm::dvec2> getPoints() const;
		void updateByNewPoint(const glm::dvec2& point, bool shiftPressed);
		bool hit(const glm::dvec2& point) const;
		void resize(const glm::dvec2& scale, const glm::dvec2& resize_center);
		BoundingBox boundingBox() const;
	};

}