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
		Polygon25D polygon;
		QMap<int, bool> neighbors;

	public:
		BodyGeometry(boost::shared_ptr<Joint> pivot1, boost::shared_ptr<Joint> pivot2, const Polygon25D& polygon) : pivot1(pivot1), pivot2(pivot2), polygon(polygon) {}

		std::vector<glm::dvec2> getActualPoints();
		void draw(QPainter& painter, const QPointF& origin, float scale);
	};

}