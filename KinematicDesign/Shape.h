#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <QPainter>
#include <boost/shared_ptr.hpp>
#include <QImage>
#include "BoundingBox.h"

namespace canvas {

	class Shape {
	public:
		static enum { RESIZE_TOP_LEFT = 0, RESIZE_TOP_RIGHT, RESIZE_BOTTOM_LEFT, RESIZE_BOTTOM_RIGHT };

	protected:
		bool selected;
		bool currently_drawing;
		glm::dvec2 origin;
		glm::dmat4x4 model_mat;
		QTransform transform;
		static QImage rotation_marker;

	public:
		Shape();
		~Shape();

		virtual boost::shared_ptr<Shape> clone() = 0;
		virtual void draw(QPainter& painter) const = 0;
		virtual void addPoint(const glm::dvec2& point) = 0;
		virtual void updateByNewPoint(const glm::dvec2& point) = 0;
		void select();
		void unselect();
		bool isSelected() const;
		void complete();
		virtual bool hit(const glm::dvec2& point) const = 0;
		void translate(const glm::dvec2& vec);
		virtual void resize(const glm::dvec2& scale, int resize_type) = 0;
		void rotate(double angle);
		glm::dvec2 getCenter() const;
		virtual BoundingBox boundingBox() const = 0;
		glm::dvec2 getRotationMarkerPosition() const;
		glm::dvec2 localCoordinate(const glm::dvec2& point) const;
		glm::dvec2 worldCoordinate(const glm::dvec2& point) const;
	};

}