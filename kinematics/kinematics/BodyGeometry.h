#pragma once

#include <vector>
#include <boost/shared_ptr.hpp>
#include <glm/glm.hpp>
#include <QPainter>
#include <QMap>
#include "KinematicUtils.h"

namespace kinematics {

	class Joint;

	class BodyGeometry {
	public:
		boost::shared_ptr<Joint> pivot1;
		boost::shared_ptr<Joint> pivot2;
		Object25D polygons;
		QMap<int, bool> neighbors;

	public:
		BodyGeometry(boost::shared_ptr<Joint> pivot1, boost::shared_ptr<Joint> pivot2, const Polygon25D& polygon) : pivot1(pivot1), pivot2(pivot2), polygons(polygon) {}
		BodyGeometry(boost::shared_ptr<Joint> pivot1, boost::shared_ptr<Joint> pivot2, const Object25D& polygons) : pivot1(pivot1), pivot2(pivot2), polygons(polygons) {}

		void normalizeCoordinates(const glm::dvec2& p1, const glm::dvec2& p2);
		std::vector<std::vector<glm::dvec2>> getActualPoints();
		size_t size() const;
		std::vector<glm::dvec2> getActualPoints(int index);
		std::vector<glm::dvec2> getActualPoints2(int index);
		void draw(QPainter& painter, const QPointF& origin, float scale);
	};

}