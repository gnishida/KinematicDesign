#include "KinematicUtils.h"
#include <random>
#include "clipper.hpp"

namespace kinematics {

	std::default_random_engine rnd_generator;

	double genRand() {
		return rand() / (float(RAND_MAX) + 1);
		//std::uniform_real_distribution<double> distribution(0, 1);
		//return distribution(rnd_generator);
	}

	double genRand(double a, double b) {
		return genRand() * (b - a) + a;
	}

	double genNormal() {
		return genNormal(0.0, 1.0);
	}

	double genNormal(double myu, double sigma) {
		if (sigma == 0) return myu;
		std::normal_distribution<double> distribution(myu, sigma);
		return distribution(rnd_generator);
	}

	/**
	 * Find the intersection on the right if you look at cener 2 from center 1.
	 *     c2
	 *     |
	 *     |   (Intersection)
	 *     |
	 *     c1
	 */
	glm::dvec2 circleCircleIntersection(const glm::dvec2& center1, double radius1, const glm::dvec2& center2, double radius2) {
		glm::dvec2 dir = center2 - center1;
		double d = glm::length(dir);
		if (d > radius1 + radius2 || d < abs(radius1 - radius2)) {
			if (d <= radius1 + radius2 + TOL && d > radius1 + radius2) {
				return center1 + dir / (radius1 + radius2) * radius1;
				//d = radius1 + radius2;
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
		contour poly1 = polygon1;
		boost::geometry::correct(poly1);
		contour poly2 = polygon2;
		boost::geometry::correct(poly2);

		std::deque<contour> output;
		boost::geometry::intersection(poly1, poly2, output);
		if (output.size() > 0) return true;
		else return false;
	}

	bool lineLineIntersection(const glm::dvec2& a, const glm::dvec2& u, const glm::dvec2& b, const glm::dvec2& v, glm::dvec2& intPoint) {
		if (glm::length(u) < TOL || glm::length(u) < TOL) return false;

		double numer = v.x*(b.y - a.y) + v.y*(a.x - b.x);
		double denom = u.y*v.x - u.x*v.y;

		if (denom == 0.0) return false;

		double t0 = numer / denom;

		intPoint = a + t0 * u;
		return true;
	}

	bool segmentSegmentIntersection(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c, const glm::dvec2& d, glm::dvec2& intPoint) {
		glm::dvec2 u = b - a;
		glm::dvec2 v = d - c;

		if (glm::length(u) < TOL || glm::length(u) < TOL) {
			return false;
		}

		double numer = v.x*(c.y - a.y) + v.y*(a.x - c.x);
		double denom = u.y*v.x - u.x*v.y;

		if (denom == 0.0) return false;

		double t0 = numer / denom;

		glm::dvec2 ipt = a + u * t0;
		glm::dvec2 tmp = ipt - c;
		double t1;
		if (glm::dot(tmp, v) > 0.0) {
			t1 = glm::length(tmp) / glm::length(v);
		}
		else {
			t1 = -1.0 * glm::length(tmp) / glm::length(v);
		}

		// Check if intersection is within segments
		if (!((t0 >= TOL) && (t0 <= 1.0f - TOL) && (t1 >= TOL) && (t1 <= 1.0f - TOL))) {
			return false;
		}

		intPoint = a + (b - a) * t0;

		return true;
	}

	/**
	 * Calculate the distance from the segment a-b to point c.
	 */
	double distanceToSegment(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c) {
		float r_numerator = (c.x - a.x) * (b.x - a.x) + (c.y - a.y) * (b.y - a.y);
		float r_denomenator = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);

		if (r_denomenator <= 0.0) {
			return glm::length(a - c);
		}

		float r = r_numerator / r_denomenator;

		if (r < 0 || r > 1) {
			return std::min(glm::length(c - a), glm::length(c - b));
		}
		else {
			return abs((a.y - c.y) * (b.x - a.x) - (a.x - c.x) * (b.y - a.y)) / sqrt(r_denomenator);
		}
	}

	glm::dvec2 circleCenterFromThreePoints(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c) {
		glm::dvec2 m1 = (a + b) * 0.5;
		glm::dvec2 v1 = b - a;
		v1 /= glm::length(v1);
		glm::dvec2 h1(-v1.y, v1.x);

		glm::dvec2 m2 = (b + c) * 0.5;
		glm::dvec2 v2 = c - b;
		v2 /= glm::length(v2);
		glm::dvec2 h2(-v2.y, v2.x);

		glm::dvec2 intPt;
		if (!lineLineIntersection(m1, h1, m2, h2, intPt)) {
			throw "No circle center";
		}

		return intPt;
	}

	/**
	* Given three points, a, b, and c, find the coordinates of point p1, such that
	* length a-p1 is l0, length b-p2 is l1, length c-p3 is l2, length p1-p2 is r0, length p1-p3 is r1, length p2-p3 is r2.
	* Also, p1 should be close to prev_pos, p2 should be close to prev_pos2, p3 should be close to prev_pos3.
	*/
	glm::dvec2 threeLengths(const glm::dvec2& a, double l0, const glm::dvec2& b, double l1, const glm::dvec2& c, double l2, double r0, double r1, double r2, const glm::dvec2& prev_pos, const glm::dvec2& prev_pos2, const glm::dvec2& prev_pos3) {
		double min_dist = std::numeric_limits<double>::max();
		glm::dvec2 best_pos;

		double theta0 = 0;
		double theta1 = M_PI * 2;
		double delta_theta = 0.01;
		double best_theta;
		for (int i = 0; i < 10; i++) {
			best_theta = threeLengths(a, l0, b, l1, c, l2, r0, r1, r2, prev_pos, prev_pos2, prev_pos3, theta0, theta1, delta_theta);

			theta0 = best_theta - delta_theta * 2;
			theta1 = best_theta + delta_theta * 2;
			delta_theta *= 0.1;
		}

		glm::dvec2 pos(a.x + l0 * cos(best_theta), a.y + l0 * sin(best_theta));

		double dist = std::numeric_limits<double>::max();
		try {
			glm::dvec2 P1a = circleCircleIntersection(pos, r0, b, l1);// , prev_pos2);
			glm::dvec2 P1b = circleCircleIntersection(b, l1, pos, r0);
			glm::dvec2 P2a = circleCircleIntersection(pos, r1, c, l2);// , prev_pos3);
			glm::dvec2 P2b = circleCircleIntersection(c, l2, pos, r1);

			if (glm::length(P1a - prev_pos2) < l1 * 0.5 && glm::length(P2a - prev_pos3) < l2 * 0.5) {
				dist = std::min(dist, std::abs(glm::length(P1a - P2a) - r2));
			}
			if (glm::length(P1a - prev_pos2) < l1 * 0.5 && glm::length(P2b - prev_pos3) < l2 * 0.5) {
				dist = std::min(dist, std::abs(glm::length(P1a - P2b) - r2));
			}
			if (glm::length(P1b - prev_pos2) < l1 * 0.5 && glm::length(P2a - prev_pos3) < l2 * 0.5) {
				dist = std::min(dist, std::abs(glm::length(P1b - P2a) - r2));
			}
			if (glm::length(P1b - prev_pos2) < l1 * 0.5 && glm::length(P2b - prev_pos3) < l2 * 0.5) {
				dist = std::min(dist, std::abs(glm::length(P1b - P2b) - r2));
			}
		}
		catch (char* ex) {
		}

		if (dist > 0.001) throw "No solution";

		return pos;
	}

	double threeLengths(const glm::dvec2& a, double l0, const glm::dvec2& b, double l1, const glm::dvec2& c, double l2, double r0, double r1, double r2, const glm::dvec2& prev_pos, const glm::dvec2& prev_pos2, const glm::dvec2& prev_pos3, double theta0, double theta1, double delta_theta) {
		double min_dist = std::numeric_limits<double>::max();
		double best_theta;

		// calculate angle sign of b-prev_pos2-prev_pos
		bool prev_sign2 = crossProduct(prev_pos2 - b, prev_pos - prev_pos2) >= 0 ? true : false;

		// calculate angle sign of c-prev_pos3-prev_pos
		bool prev_sign3 = crossProduct(prev_pos3 - c, prev_pos - prev_pos3) >= 0 ? true : false;

		for (double theta = theta0; theta <= theta1; theta += delta_theta) {
			glm::dvec2 pos(a.x + l0 * cos(theta), a.y + l0 * sin(theta));
			if (glm::length(pos - prev_pos) > l0 * 0.5) continue;

			try {
				glm::dvec2 P1a = circleCircleIntersection(pos, r0, b, l1);// , prev_pos2);
				glm::dvec2 P1b = circleCircleIntersection(b, l1, pos, r0);
				glm::dvec2 P2a = circleCircleIntersection(pos, r1, c, l2);// , prev_pos3);
				glm::dvec2 P2b = circleCircleIntersection(c, l2, pos, r1);

				// calculate angle sign of b-P1-pos
				bool sign2a = crossProduct(P1a - b, pos - P1a) >= 0 ? true : false;
				bool sign2b = crossProduct(P1b - b, pos - P1b) >= 0 ? true : false;

				// calculate angle sign of c-P2-pos
				bool sign3a = crossProduct(P2a - c, pos - P2a) >= 0 ? true : false;
				bool sign3b = crossProduct(P2b - c, pos - P2b) >= 0 ? true : false;

				double dist = std::numeric_limits<double>::max();
				if (glm::length(P1a - prev_pos2) < l1 * 0.5 && glm::length(P2a - prev_pos3) < l2 * 0.5 && sign2a == prev_sign2 || sign3a == prev_sign3) {
					dist = std::min(dist, std::abs(glm::length(P1a - P2a) - r2));
				}
				if (glm::length(P1a - prev_pos2) < l1 * 0.5 && glm::length(P2b - prev_pos3) < l2 * 0.5 && sign2a == prev_sign2 || sign3b == prev_sign3) {
					dist = std::min(dist, std::abs(glm::length(P1a - P2b) - r2));
				}
				if (glm::length(P1b - prev_pos2) < l1 * 0.5 && glm::length(P2a - prev_pos3) < l2 * 0.5 && sign2b == prev_sign2 || sign3a == prev_sign3) {
					dist = std::min(dist, std::abs(glm::length(P1b - P2a) - r2));
				}
				if (glm::length(P1b - prev_pos2) < l1 * 0.5 && glm::length(P2b - prev_pos3) < l2 * 0.5 && sign2b == prev_sign2 || sign3b == prev_sign3) {
					dist = std::min(dist, std::abs(glm::length(P1b - P2b) - r2));
				}

				if (dist < min_dist) {
					min_dist = dist;
					best_theta = theta;
				}
			}
			catch (char* ex) {
			}
		}

		return best_theta;
	}

	/**
	 * Calculate the circle center in a least square manner.
	 * Follow the algorithm described in http://www.dtcenter.org/met/users/docs/write_ups/circle_fit.pdf
	 */
	void circleFit(const std::vector<glm::dvec2>& p, glm::dvec2& c, double& r) {
		if (p.size() == 0) throw "No point";
		else if (p.size() == 1) {
			c = p[0];
			r = 0;
		}
		else if (p.size() == 2) {
			c = (p[0] + p[1]) * 0.5;
			r = glm::length(p[0] - p[1]) * 0.5;
		}
		else if (p.size() == 3) {
			c = circleCenterFromThreePoints(p[0], p[1], p[2]);
			r = glm::length(p[0] - c);
		}
		else {
			// calculate the average x and y
			glm::dvec2 avg;
			for (int i = 0; i < p.size(); i++) {
				avg += p[i];
			}
			avg /= p.size();

			// convert to uv coordinates
			std::vector<glm::dvec2> uv(p.size());
			for (int i = 0; i < p.size(); i++) {
				uv[i] = p[i] - avg;
			}

			// calculate S_uu, S_uv, S_vv, S_uuu, S_uuv, S_uvv, and S_vvv
			double S_uu = 0;
			double S_uv = 0;
			double S_vv = 0;
			double S_uuu = 0;
			double S_uuv = 0;
			double S_uvv = 0;
			double S_vvv = 0;
			for (int i = 0; i < uv.size(); i++) {
				S_uu += uv[i].x * uv[i].x;
				S_uv += uv[i].x * uv[i].y;
				S_vv += uv[i].y * uv[i].y;
				S_uuu += uv[i].x * uv[i].x * uv[i].x;
				S_uuv += uv[i].x * uv[i].x * uv[i].y;
				S_uvv += uv[i].x * uv[i].y * uv[i].y;
				S_vvv += uv[i].y * uv[i].y * uv[i].y;
			}

			// calculate the det of mat [S_uu, S_uv; S_uv, S_vv]
			double det = S_uu * S_vv - S_uv * S_uv;
			if (det == 0) {
				throw "No circle center.";
			}

			// calculate the circle center in uv coordinates
			double uc = (S_vv * (S_uuu + S_uvv) - S_uv * (S_uuv + S_vvv)) * 0.5 / det;
			double vc = (-S_uv * (S_uuu + S_uvv) + S_uu * (S_uuv + S_vvv)) * 0.5 / det;

			// calculate the circle center in xy coordinates
			c.x = uc + avg.x;
			c.y = vc + avg.y;

			// calculate the radius
			r = sqrt(uc * uc + vc * vc + (S_uu + S_vv) / p.size());
		}
	}

	/**
	* Find the closest point of a polygon from the specified point.
	*/
	glm::dvec2 closestOffsetPoint(const std::vector<glm::dvec2>& points, const glm::dvec2& p, double margin, int num_samples) {
		const double scale = 100;

		ClipperLib::Path subj;
		for (int i = 0; i < points.size(); i++) {
			subj.push_back(ClipperLib::IntPoint(points[i].x * scale, points[i].y * scale));
		}

		ClipperLib::Paths solution;
		ClipperLib::ClipperOffset co;
		co.AddPath(subj, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
		co.Execute(solution, -margin * scale);

		double min_dist = std::numeric_limits<double>::max();
		glm::dvec2 ans;
		for (int i = 0; i < solution.size(); i++) {
			std::vector<glm::dvec2> offset_points;
			for (int j = 0; j < solution[i].size(); j++) {
				double x = (double)solution[i][j].X / scale;
				double y = (double)solution[i][j].Y / scale;
				offset_points.push_back(glm::dvec2(x, y));
			}

			double dist;
			glm::dvec2 a = closestPoint(offset_points, p, dist, num_samples);

			if (dist < min_dist) {
				min_dist = dist;
				ans = a;
			}
		}

		if (min_dist == std::numeric_limits<double>::max()) throw "No closest point found.";

		return ans;
	}

	/**
	 * Find the closest point of a polygon from the specified point.
	 */
	glm::dvec2 closestPoint(const std::vector<glm::dvec2>& points, const glm::dvec2& p, double& dist, int num_samples) {
		std::vector<double> edge_lengths(points.size());
		double length = 0.0;
		for (int i = 0; i < points.size(); i++) {
			int next = (i + 1) % points.size();
			edge_lengths[i] = glm::length(points[next] - points[i]);
			length += edge_lengths[i];
		}

		double step = length / num_samples;

		// sample #num_samples points on the polygon
		std::vector<glm::dvec2> sampled_points(num_samples);
		int index = 0;
		double len = 0;
		for (int i = 0; i < num_samples; i++) {
			int next = (index + 1) % points.size();
			sampled_points[i] = glm::mix(points[index], points[next], len / edge_lengths[index]);

			len += step;
			if (len > edge_lengths[index]) {
				while (index < points.size() && len > edge_lengths[index]) {
					len -= edge_lengths[index];
					index++;
				}
			}
		}

		// find the cloest point
		dist = std::numeric_limits<double>::max();
		glm::dvec2 ans;
		for (int i = 0; i < num_samples; i++) {
			double d = glm::length(sampled_points[i] - p);
			if (d < dist) {
				dist = d;
				ans = sampled_points[i];
			}
		}

		return ans;
	}

	/**
	 * Calcualte the reflection point of p about the line that passes a and its direction is v.
	 */
	glm::dvec2 reflect(const glm::dvec2& p, const glm::dvec2& a, const glm::dvec2& v) {
		glm::dvec2 norm_v = v / glm::length(v);

		glm::dvec2 u = a - p;
		return p + (u - norm_v * glm::dot(u, norm_v)) * 2.0;
	}

	double crossProduct(const glm::dvec2& v1, const glm::dvec2& v2) {
		return v1.x * v2.y - v1.y * v2.x;
	}

	/**
	* Create a transformation matrix that transforms the origin to O, and (1, 0) to X.
	*/
	glm::dmat3x3 calculateTransMatrix(const glm::dvec2& O, const glm::dvec2& X) {
		double angle = atan2(X.y - O.y, X.x - O.x);
		glm::dmat3x3 T;
		T[0][0] = cos(angle);
		T[0][1] = sin(angle);
		T[0][2] = 0;
		T[1][0] = -sin(angle);
		T[1][1] = cos(angle);
		T[1][2] = 0;
		T[2][0] = O.x;
		T[2][1] = O.y;
		T[2][2] = 1;
		return T;
	}

	/**
	* Given that point p1 goes to p2, and the point q1 goes to q2,
	* return the matrix for this transformation.
	*/
	glm::dmat3x3 affineTransform(const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& q1, const glm::dvec2& q2) {
		double theta1 = atan2(q1.y - p1.y, q1.x - p1.x);
		double theta2 = atan2(q2.y - p2.y, q2.x - p2.x);
		double theta = theta2 - theta1;

		return glm::dmat3x3({ cos(theta), sin(theta), 0, -sin(theta), cos(theta), 0, -p1.x * cos(theta) + p1.y * sin(theta) + p2.x, -p1.x * sin(theta) - p1.y * cos(theta) + p2.y, 1 });
	}

	double area(const std::vector<glm::dvec2>& points) {
		contour poly = points;
		boost::geometry::correct(poly);
		return boost::geometry::area(poly);
	}

	bool withinPolygon(const std::vector<glm::dvec2>& points, const glm::dvec2& pt) {
		contour poly = points;
		boost::geometry::correct(poly);
		return boost::geometry::within(pt, poly);
	}

	bool withinPolygon(const std::vector<std::vector<glm::dvec2>>& polygons, const glm::dvec2& pt) {
		for (int i = 0; i < polygons.size(); i++) {
			if (withinPolygon(polygons[i], pt)) return true;
		}
		return false;
	}

	BBox boundingBox(const std::vector<glm::dvec2>& points) {
		double min_x = std::numeric_limits<double>::max();
		double max_x = -std::numeric_limits<double>::max();
		double min_y = std::numeric_limits<double>::max();
		double max_y = -std::numeric_limits<double>::max();
		for (int i = 0; i < points.size(); i++) {
			min_x = std::min(min_x, points[i].x);
			max_x = std::max(max_x, points[i].x);
			min_y = std::min(min_y, points[i].y);
			max_y = std::max(max_y, points[i].y);
		}
		return BBox(glm::dvec2(min_x, min_y), glm::dvec2(max_x, max_y));
	}

	std::vector<std::vector<glm::dvec2>> unionPolygon(std::vector<std::vector<glm::dvec2>> polygons) {
		if (polygons.size() <= 1) return polygons;

		std::vector<polygon> plgns(polygons.size());
		for (int i = 0; i < polygons.size(); i++) {
			boost::geometry::assign_points(plgns[i], polygons[i]);
			boost::geometry::correct(plgns[i]);
		}

		bool updated = true;
		while (updated) {
			updated = false;
			for (int i = 0; i < plgns.size(); i++) {
				bool intersected = false;
				int index;
				for (int j = i + 1; j < plgns.size(); j++) {
					if (boost::geometry::intersects(plgns[i], plgns[j])) {
						intersected = true;
						index = j;
						break;
					}
				}

				if (intersected) {
					std::vector<polygon> marged;
					boost::geometry::union_(plgns[i], plgns[index], marged);
					plgns.erase(plgns.begin() + index);
					plgns.erase(plgns.begin() + i);
					plgns.insert(plgns.end(), marged.begin(), marged.end());

					updated = true;
					break;
				}
			}
		}

		std::vector<std::vector<glm::dvec2>> ans(plgns.size());
		for (int i = 0; i < plgns.size(); i++) {
			for (int j = 0; j < plgns[i].outer().size(); j++) {
				ans[i].push_back(glm::dvec2(plgns[i].outer()[j].x(), plgns[i].outer()[j].y()));
			}
			if (ans[i].back() == ans[i].front()) ans[i].pop_back();
		}

		return ans;
	}

	std::vector<glm::dvec2> generateBarPolygon(const glm::dvec2& p1, const glm::dvec2& p2, float link_width) {
		std::vector<glm::dvec2> ans;

		glm::dvec2 vec = glm::normalize(p2 - p1);
		glm::dvec2 perp(-vec.y, vec.x);
		perp *= link_width * 0.5;

		ans.push_back(p1 + perp);
		ans.push_back(p1 - perp);
		ans.push_back(p2 - perp);
		ans.push_back(p2 + perp);

		return ans;
	}

	std::vector<glm::dvec2> generateRoundedBarPolygon(const glm::dvec2& p1, const glm::dvec2& p2, float link_radius, int num_slices) {
		std::vector<glm::dvec2> ans;

		double theta0 = atan2(p2.y - p1.y, p2.x - p1.x) + M_PI * 0.5;
		for (int k = 0; k <= num_slices / 2; k++) {
			if (p1 != p2 || k < num_slices / 2) {
				double theta = theta0 + M_PI * 2 / num_slices * k;
				ans.push_back(glm::dvec2(cos(theta) * link_radius + p1.x, sin(theta) * link_radius + p1.y));
			}
		}
		theta0 += M_PI;
		for (int k = 0; k <= num_slices / 2; k++) {
			if (p1 != p2 || k < num_slices / 2) {
				double theta = theta0 + M_PI * 2 / num_slices * k;
				ans.push_back(glm::dvec2(cos(theta) * link_radius + p2.x, sin(theta) * link_radius + p2.y));
			}
		}

		return ans;
	}

	std::vector<glm::dvec2> generateCirclePolygon(const glm::dvec2& p, float radius, int num_slices) {
		std::vector<glm::dvec2> ans;

		for (int k = 0; k < num_slices; k++) {
			double theta = M_PI * 2 / num_slices * k;
			ans.push_back(glm::dvec2(cos(theta) * radius + p.x, sin(theta) * radius + p.y));
		}
		return ans;
	}

}