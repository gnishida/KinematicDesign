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
		BodyGeometry(boost::shared_ptr<Joint> pivot1, boost::shared_ptr<Joint> pivot2, const Polygon25D& polygon);
		BodyGeometry(boost::shared_ptr<Joint> pivot1, boost::shared_ptr<Joint> pivot2, const Object25D& polygons);

		glm::dvec2 worldToLocal(const glm::dvec2& pt);
		glm::dvec2 localToWorld(const glm::dvec2& pt);
		std::vector<std::vector<glm::dvec2>> getActualPoints();
		size_t size() const;
		std::vector<glm::dvec2> getActualPoints(int index);
		std::vector<glm::dvec2> getActualPoints2(int index);
		void draw(QPainter& painter, const QPointF& origin, float scale);
			
		glm::dmat3x2 getLocalToWorldModel();
		glm::dmat3x2 getWorldToLocalModel();
	};

}