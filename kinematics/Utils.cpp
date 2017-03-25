#include "Utils.h"

namespace kinematics {

	glm::dvec2 rotatePoint(const glm::dvec2& pt, double angle) {
		return glm::dvec2(cos(angle) * pt.x - sin(angle) * pt.y, sin(angle) * pt.x + cos(angle) * pt.y);
	}

	glm::dvec2 rotatePoint(const glm::dvec2& pt, double angle, const glm::dvec2& rotation_center) {
		return glm::dvec2(cos(angle) * (pt.x - rotation_center.x) - sin(angle) * (pt.y - rotation_center.y) + rotation_center.x,
			sin(angle) * (pt.x - rotation_center.x) + cos(angle) * (pt.y - rotation_center.y) + rotation_center.y);
	}

	glm::dvec2 circleCircleIntersection(const glm::dvec2& center1, double radius1, const glm::dvec2& center2, double radius2) {
		glm::dvec2 dir = center2 - center1;
		double d = glm::length(dir);
		if (d > radius1 + radius2 || d < abs(radius1 - radius2)) {
			if (d <= radius1 + radius2 + TOL && d > radius1 + radius2) {
				d = radius1 + radius2;
			}
			else if (d >= abs(radius1 - radius2) - TOL && d < abs(radius1 - radius2)) {
				d = abs(radius1 - radius2);
			}
			else {
				throw "No intersection";
			}
		}

		double a = (radius1 * radius1 - radius2 * radius2 + d * d) / d / 2.0;
		double h = sqrt(radius1 * radius1 - a * a);

		glm::dvec2 perp(dir.y, -dir.x);
		perp /= glm::length(perp);

		return center1 + dir * a / d + perp * h;
	}

	glm::dvec2 circleCircleIntersection(const glm::dvec2& center1, double radius1, const glm::dvec2& center2, double radius2, const glm::dvec2& prev_int) {
		glm::dvec2 dir = center2 - center1;
		double d = glm::length(dir);
		if (d > radius1 + radius2 || d < abs(radius1 - radius2)) {
			if (d <= radius1 + radius2 + TOL && d > radius1 + radius2) {
				d = radius1 + radius2;
			}
			else if (d >= abs(radius1 - radius2) - TOL && d < abs(radius1 - radius2)) {
				d = abs(radius1 - radius2);
			}
			else {
				throw "No intersection";
			}
		}

		double a = (radius1 * radius1 - radius2 * radius2 + d * d) / d / 2.0;
		double h = sqrt(radius1 * radius1 - a * a);

		glm::dvec2 perp(dir.y, -dir.x);
		perp /= glm::length(perp);

		glm::dvec2 result1 = center1 + dir * a / d + perp * h;
		glm::dvec2 result2 = center1 + dir * a / d - perp * h;
		if (glm::length(result1 - prev_int) <= glm::length(result2 - prev_int)) {
			return result1;
		}
		else {
			return result2;
		}
	}

	/**
	 * Find the intersection that is further from p1
	 */
	glm::dvec2 circleLineIntersection(const glm::dvec2& center, double radius, const glm::dvec2& p1, const glm::dvec2& p2) {
		glm::dvec2 dir = p2 - p1;
		dir /= glm::length(dir);

		glm::dvec2 n(-dir.y, dir.x);

		double d = glm::dot(p1 - center, n);
		double h = sqrt(radius * radius - d * d);

		if (abs(d) > radius) {
			throw "No intersection";
		}

		return center + n * d - dir * h;
	}

	/**
	* Find the intersection that is closer to prev_int
	*/
	glm::dvec2 circleLineIntersection(const glm::dvec2& center, double radius, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& prev_int) {
		glm::dvec2 dir = p2 - p1;
		dir /= glm::length(dir);

		glm::dvec2 n(-dir.y, dir.x);

		double d = glm::dot(p1 - center, n);
		double h = sqrt(radius * radius - d * d);

		if (abs(d) > radius) {
			throw "No intersection";
		}

		glm::dvec2 result1 = center + n * d + dir * h;
		glm::dvec2 result2 = center + n * d - dir * h;
		if (glm::length(result1 - prev_int) <= glm::length(result2 - prev_int)) {
			return result1;
		}
		else {
			return result2;
		}
	}

	bool polygonPolygonIntersection(const std::vector<glm::dvec2>& polygon1, const std::vector<glm::dvec2>& polygon2) {
		// make the polygons closed
		std::vector<glm::dvec2> closed_polygon1 = polygon1;
		if (closed_polygon1.size() == 0) return false;
		if (closed_polygon1.front() != closed_polygon1.back()) closed_polygon1.push_back(closed_polygon1.front());
		std::vector<glm::dvec2> closed_polygon2 = polygon2;
		if (closed_polygon2.size() == 0) return false;
		if (closed_polygon2.front() != closed_polygon2.back()) closed_polygon2.push_back(closed_polygon2.front());

		std::deque<polygon> output;
		boost::geometry::intersection(closed_polygon1, closed_polygon2, output);

		if (output.size() > 0) return true;
		else return false;
	}

	bool pointWithinPolygon(const glm::dvec2& pt, const std::vector<glm::dvec2>& polygon) {
		boost::geometry::model::polygon<glm::dvec2> closed_polygon;
		boost::geometry::assign_points(closed_polygon, polygon);
		boost::geometry::correct(closed_polygon);
		return boost::geometry::within(pt, closed_polygon);
	}

}