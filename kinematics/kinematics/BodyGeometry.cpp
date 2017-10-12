#include "BodyGeometry.h"
#include "Joint.h"
#include "KinematicUtils.h"
#include <glm/gtc/matrix_transform.hpp>

namespace kinematics {

	BodyGeometry::BodyGeometry(boost::shared_ptr<Joint> pivot1, boost::shared_ptr<Joint> pivot2, const Polygon25D& polygon) : pivot1(pivot1), pivot2(pivot2), polygons(polygon) {
	}

	BodyGeometry::BodyGeometry(boost::shared_ptr<Joint> pivot1, boost::shared_ptr<Joint> pivot2, const Object25D& polygons) : pivot1(pivot1), pivot2(pivot2), polygons(polygons) {
	}

	/**
	 * Convert the coordinates to the local coordinate system.
	 */
	glm::dvec2 BodyGeometry::worldToLocal(const glm::dvec2& pt) {
		return getWorldToLocalModel() * glm::dvec3(pt, 1);
	}

	/**
	 * Convert the local coordinates to the world coordinate system.
	 */
	glm::dvec2 BodyGeometry::localToWorld(const glm::dvec2& pt) {
		return getLocalToWorldModel() * glm::dvec3(pt, 1);
	}

	/**
	 * Get the actual coordinates of the body geometry.
	 * Note that "points" store the original coordinates in the model coordinate system.
	 */
	std::vector<std::vector<glm::dvec2>> BodyGeometry::getActualPoints() {
		std::vector<std::vector<glm::dvec2>> actual_points(polygons.size());

		glm::dmat3x2 model = getLocalToWorldModel();

		for (int i = 0; i < polygons.size(); i++) {
			for (int k = 0; k < polygons[i].points.size(); ++k) {
				actual_points[i].push_back(model * glm::dvec3(polygons[i].points[k], 1));
			}
		}

		return actual_points;
	}

	size_t BodyGeometry::size() const {
		return polygons.size();
	}

	std::vector<glm::dvec2> BodyGeometry::getActualPoints(int index) {
		std::vector<glm::dvec2> actual_points;

		glm::dmat3x2 model = getLocalToWorldModel();

		for (int k = 0; k < polygons[index].points.size(); ++k) {
			actual_points.push_back(model * glm::dvec3(polygons[index].points[k], 1));
		}

		return actual_points;
	}

	/**
	 * This is a hacky implementation.
	 * For a frustram shape, we store additional polygons for the top shape.
	 * This function returns the actual coordinates of the top face.
	 */
	std::vector<glm::dvec2> BodyGeometry::getActualPoints2(int index) {
		if (polygons[index].points2.size() == 0) return getActualPoints(index);

		std::vector<glm::dvec2> actual_points;

		glm::dmat3x2 model = getLocalToWorldModel();

		for (int k = 0; k < polygons[index].points2.size(); ++k) {
			actual_points.push_back(model * glm::dvec3(polygons[index].points2[k], 1));
		}

		return actual_points;
	}

	void BodyGeometry::draw(QPainter& painter, const QPointF& origin, float scale) {
		painter.save();

		painter.setPen(QPen(QColor(0, 0, 0), 1));
		painter.setBrush(QBrush(QColor(0, 255, 0, 60)));
		std::vector<std::vector<glm::dvec2>> actual_points = getActualPoints();
		for (int i = 0; i < actual_points.size(); i++) {
			QPolygonF pts;
			for (int k = 0; k < actual_points[i].size(); ++k) {
				pts.push_back(QPointF(origin.x() + actual_points[i][k].x * scale, origin.y() - actual_points[i][k].y * scale));
			}
			painter.drawPolygon(pts);
		}

		painter.restore();
	}

	glm::dmat3x2 BodyGeometry::getLocalToWorldModel() {
		glm::dvec2 dir = pivot2->pos - pivot1->pos;
		double angle = atan2(dir.y, dir.x);
		glm::dvec2 p1 = pivot1->pos;

		glm::dmat3x2 model;
		model[0][0] = cos(angle);
		model[1][0] = -sin(angle);
		model[2][0] = p1.x;
		model[0][1] = sin(angle);
		model[1][1] = cos(angle);
		model[2][1] = p1.y;
		return model;
	}

	glm::dmat3x2 BodyGeometry::getWorldToLocalModel() {
		glm::vec2 dir = pivot2->pos - pivot1->pos;
		double angle = -atan2(dir.y, dir.x);

		glm::dmat3x2 model;
		model[0][0] = cos(angle);
		model[1][0] = -sin(angle);
		model[2][0] = -pivot1->pos.x * cos(angle) + pivot1->pos.y * sin(angle);
		model[0][1] = sin(angle);
		model[1][1] = cos(angle);
		model[2][1] = -pivot1->pos.x * sin(angle) - pivot1->pos.y * cos(angle);
		return model;
	}

}