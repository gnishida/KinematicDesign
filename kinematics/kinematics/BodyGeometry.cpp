#include "BodyGeometry.h"
#include "Joint.h"
#include "KinematicUtils.h"

namespace kinematics {

	/**
	 * Get the actual coordinates of the body geometry.
	 * Note that "points" store the original coordinates in the model coordinate system.
	 */
	std::vector<std::vector<glm::dvec2>> BodyGeometry::getActualPoints() {
		std::vector<std::vector<glm::dvec2>> actual_points(polygons.size());

		glm::dvec2 dir = pivot2->pos - pivot1->pos;
		double angle = atan2(dir.y, dir.x);
		//glm::dvec2 p1 = (bodies[i].pivot1->pos + bodies[i].pivot2->pos) * 0.5;
		glm::dvec2 p1 = pivot1->pos;

		glm::dmat3x2 mat;
		mat[0][0] = cos(angle);
		mat[1][0] = -sin(angle);
		mat[2][0] = p1.x;
		mat[0][1] = sin(angle);
		mat[1][1] = cos(angle);
		mat[2][1] = p1.y;

		for (int i = 0; i < polygons.size(); i++) {
			for (int k = 0; k < polygons[i].points.size(); ++k) {
				glm::dvec2 actual_point = mat * glm::dvec3(polygons[i].points[k], 1);
				actual_points[i].push_back(actual_point);
			}
		}

		return actual_points;
	}

	size_t BodyGeometry::size() const {
		return polygons.size();
	}

	std::vector<glm::dvec2> BodyGeometry::getActualPoints(int index) {
		std::vector<glm::dvec2> actual_points;

		glm::dvec2 dir = pivot2->pos - pivot1->pos;
		double angle = atan2(dir.y, dir.x);
		glm::dvec2 p1 = pivot1->pos;

		glm::dmat3x2 mat;
		mat[0][0] = cos(angle);
		mat[1][0] = -sin(angle);
		mat[2][0] = p1.x;
		mat[0][1] = sin(angle);
		mat[1][1] = cos(angle);
		mat[2][1] = p1.y;

		for (int k = 0; k < polygons[index].points.size(); ++k) {
			glm::dvec2 actual_point = mat * glm::dvec3(polygons[index].points[k], 1);
			actual_points.push_back(actual_point);
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

		glm::dvec2 dir = pivot2->pos - pivot1->pos;
		double angle = atan2(dir.y, dir.x);
		glm::dvec2 p1 = pivot1->pos;

		glm::dmat3x2 mat;
		mat[0][0] = cos(angle);
		mat[1][0] = -sin(angle);
		mat[2][0] = p1.x;
		mat[0][1] = sin(angle);
		mat[1][1] = cos(angle);
		mat[2][1] = p1.y;

		for (int k = 0; k < polygons[index].points2.size(); ++k) {
			glm::dvec2 actual_point = mat * glm::dvec3(polygons[index].points2[k], 1);
			actual_points.push_back(actual_point);
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

}