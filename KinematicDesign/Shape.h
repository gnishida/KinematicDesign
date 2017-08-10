#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <QPainter>
#include <QDomDocument>
#include <QImage>
#include <boost/shared_ptr.hpp>
#include "BoundingBox.h"

namespace canvas {

	class Shape {
	public:
		static enum { TYPE_BODY = 0, TYPE_LINKAGE_REGION };
		static enum { RESIZE_TOP_LEFT = 0, RESIZE_TOP_RIGHT, RESIZE_BOTTOM_LEFT, RESIZE_BOTTOM_RIGHT };

	protected:
		int subtype;
		bool selected;
		bool currently_drawing;
		glm::dvec2 pos;
		double theta;
		static QImage rotation_marker;
		static std::vector<QBrush> brushes;

	public:
		Shape(int subtype);
		~Shape();

		int getSubType() { return subtype; }
		virtual boost::shared_ptr<Shape> clone() const = 0;
		virtual void draw(QPainter& painter, const QPointF& origin, double scale) const = 0;
		virtual QDomElement toXml(QDomDocument& doc) const = 0;
		glm::dmat3x3 getModelMatrix() const;
		virtual void addPoint(const glm::dvec2& point) = 0;
		virtual std::vector<glm::dvec2> getPoints() const = 0;
		virtual void updateByNewPoint(const glm::dvec2& point, bool shiftPressed) = 0;
		void select();
		void unselect();
		bool isSelected() const;
		void startDrawing();
		void completeDrawing();
		virtual bool hit(const glm::dvec2& point) const = 0;
		void translate(const glm::dvec2& vec);
		virtual void resize(const glm::dvec2& scale, const glm::dvec2& resize_center) = 0;
		void rotate(double angle);
		glm::dvec2 getCenter() const;
		virtual BoundingBox boundingBox() const = 0;
		glm::dvec2 getRotationMarkerPosition(double scale) const;
		glm::dvec2 localCoordinate(const glm::dvec2& point) const; 
		glm::dvec2 worldCoordinate(const glm::dvec2& point) const;
	};

}