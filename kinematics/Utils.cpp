#include "Utils.h"
#include <random>

namespace kinematics {

	double genRand() {
		return rand() / (float(RAND_MAX) + 1);
	}

	double genRand(double a, double b) {
		return genRand() * (b - a) + a;
	}

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

	glm::dvec2 lineLineIntersection(const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const glm::dvec2& p4) {
		glm::dvec2 u = p2 - p1;
		glm::dvec2 v = p4 - p3;

		if (glm::length(u) == 0 || glm::length(v) == 0) {
			throw "No intersection";
		}

		double numer = v.x * (p3.y - p1.y) + v.y * (p1.x - p3.x);
		double denom = u.y * v.x - u.x * v.y;

		if (denom == 0.0)  {
			throw "Non intersection";
		}

		double t0 = numer / denom;

		glm::dvec2 ipt = p1 + t0 * u;
		glm::dvec2 tmp = ipt - p3;
		double t1;
		if (glm::dot(tmp, v) > 0.0) {
			t1 = glm::length(tmp) / glm::length(v);
		}
		else {
			t1 = -1.0 * glm::length(tmp) / glm::length(v);
		}

		return p1 + (p2 - p1) * t0;
	}

	double segmentSegmentDistance(const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const glm::dvec2& p4) {
		if (isSegmentSegmentIntersected(p1, p2, p3, p4)) return 0;
		else {
			double dist1 = pointSegmentDistance(p3, p4, p1);
			double dist2 = pointSegmentDistance(p3, p4, p2);
			double dist3 = pointSegmentDistance(p1, p2, p3);
			double dist4 = pointSegmentDistance(p1, p2, p4);

			return std::min(std::min(std::min(dist1, dist2), dist3), dist4);
		}
	}

	double segmentSegmentDistance(const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const glm::dvec2& p4, glm::dvec2& q1, glm::dvec2& q2) {
		if (isSegmentSegmentIntersected(p1, p2, p3, p4)) {
			q1 = lineLineIntersection(p1, p2, p3, p4);
			q2 = q1;
			return 0;
		}
		else {
			glm::dvec2 a, b, c, d;
			double dist1 = pointSegmentDistance(p3, p4, p1, a);
			double dist2 = pointSegmentDistance(p3, p4, p2, b);
			double dist3 = pointSegmentDistance(p1, p2, p3, c);
			double dist4 = pointSegmentDistance(p1, p2, p4, d);

			if (dist1 <= dist2 && dist1 <= dist3 && dist1 <= dist4) {
				q1 = p1;
				q2 = a;
				return dist1;
			}
			else if (dist2 <= dist3 && dist2 <= dist4) {
				q1 = p2;
				q2 = b;
				return dist2;
			}
			else if (dist3 <= dist4) {
				q1 = c;
				q2 = p3;
				return dist3;
			}
			else {
				q1 = d;
				q2 = p4;
				return dist4;
			}
		}
	}

	bool isSegmentSegmentIntersected(const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const glm::dvec2& p4) {
		glm::dvec2 v1 = p2 - p1;
		glm::dvec2 a = p3 - p2;
		glm::dvec2 b = p4 - p2;
		if ((v1.x * a.y - v1.y * a.x) * (v1.x * b.y - v1.y * b.x) > 0) return false;

		glm::dvec2 v2 = p4 - p3;
		glm::dvec2 c = p1 - p4;
		glm::dvec2 d = p2 - p4;
		if ((v2.x * c.y - v2.y * c.x) * (v2.x * d.y - v2.y * d.x) > 0) return false;

		return true;
	}

	/**
	 * Return the distance from point P to line segment VW.
	 */
	double pointSegmentDistance(const glm::dvec2& v, const glm::dvec2& w, const glm::dvec2& p) {
		// Return minimum distance between line segment vw and point p
		double l2 = glm::length(v - w);  // i.e. |w-v|^2 -  avoid a sqrt
		l2 = l2 * l2;
		if (l2 == 0.0) return glm::length(p - v);   // v == w case

		// Consider the line extending the segment, parameterized as v + t (w - v).
		// We find projection of point p onto the line. 
		// It falls where t = [(p-v) . (w-v)] / |w-v|^2
		// We clamp t from [0,1] to handle points outside the segment vw.
		double t = std::max(0.0, std::min(1.0, glm::dot(p - v, w - v) / l2));
		glm::dvec2 projection = v + t * (w - v);  // Projection falls on the segment
		return glm::length(p - projection);
	}
#if 0
	double pointLineDistance(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c) {
		float dist;

		float r_numerator = (c.x - a.x)*(b.x - a.x) + (c.y - a.y)*(b.y - a.y);
		float r_denomenator = (b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y);

		// For the case that the denominator is 0.
		if (r_denomenator <= 0.0f) {
			return (a - c).length();
		}

		float r = r_numerator / r_denomenator;

		//
		float px = a.x + r*(b.x - a.x);
		float py = a.y + r*(b.y - a.y);
		//    
		float s = ((a.y - c.y)*(b.x - a.x) - (a.x - c.x)*(b.y - a.y)) / r_denomenator;

		float distanceLine = fabs(s)*sqrt(r_denomenator);

		if ((r >= 0) && (r <= 1)) {
			dist = distanceLine;
		}
		else {
			float dist1 = (c.x - a.x)*(c.x - a.x) + (c.y - a.y)*(c.y - a.y);
			float dist2 = (c.x - b.x)*(c.x - b.x) + (c.y - b.y)*(c.y - b.y);
			if (dist1 < dist2) {
				dist = sqrt(dist1);
			}
			else {
				dist = sqrt(dist2);
			}
		}

		return abs(dist);
	}
#endif

	double pointSegmentDistance(const glm::dvec2& v, const glm::dvec2& w, const glm::dvec2& p, glm::dvec2& nearestPt) {
		// Return minimum distance between line segment vw and point p
		double l2 = glm::length(v - w);  // i.e. |w-v|^2 -  avoid a sqrt
		l2 = l2 * l2;
		if (l2 == 0.0) {
			nearestPt = v;
			return glm::length(p - v);   // v == w case
		}

		// Consider the line extending the segment, parameterized as v + t (w - v).
		// We find projection of point p onto the line. 
		// It falls where t = [(p-v) . (w-v)] / |w-v|^2
		// We clamp t from [0,1] to handle points outside the segment vw.
		double t = std::max(0.0, std::min(1.0, glm::dot(p - v, w - v) / l2));
		nearestPt = v + t * (w - v);  // Projection falls on the segment
		return glm::length(p - nearestPt);
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