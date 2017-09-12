#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/core/point_type.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include "BBox.h"

namespace kinematics {

	class Polygon25D {
	public:
		std::vector<glm::dvec2> points;
		double depth1;
		double depth2;

	public:
		Polygon25D() : depth1(0), depth2(0) {}
		Polygon25D(const std::vector<glm::dvec2>& points, double depth1, double depth2) : points(points), depth1(depth1), depth2(depth2) {}
	};

	const double M_PI = 3.14159265;
	const double TOL = 0.0000001;

	double genRand();
	double genRand(double a, double b);
	double genNormal();
	double genNormal(double myu, double sigma);

	glm::dvec2 circleCircleIntersection(const glm::dvec2& center1, double radius1, const glm::dvec2& center2, double radius);
	glm::dvec2 circleCircleIntersection(const glm::dvec2& center1, double radius1, const glm::dvec2& center2, double radius, const glm::dvec2& prev_int);
	glm::dvec2 circleLineIntersection(const glm::dvec2& center, double radius, const glm::dvec2& p1, const glm::dvec2& p2);
	glm::dvec2 circleLineIntersection(const glm::dvec2& center, double radius, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& prev_int);
	bool polygonPolygonIntersection(const std::vector<glm::dvec2>& polygon1, const std::vector<glm::dvec2>& polygon2);
	bool lineLineIntersection(const glm::dvec2& a, const glm::dvec2& u, const glm::dvec2& b, const glm::dvec2& v, glm::dvec2& intPoint);
	bool segmentSegmentIntersection(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c, const glm::dvec2& d, glm::dvec2& intPoint);
	glm::dvec2 circleCenterFromThreePoints(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c);
	glm::dvec2 threeLengths(const glm::dvec2& a, double l0, const glm::dvec2& b, double l1, const glm::dvec2& c, double l2, double r0, double r1, double r2, const glm::dvec2& prev_pos, const glm::dvec2& prev_pos2, const glm::dvec2& prev_pos3);
	double threeLengths(const glm::dvec2& a, double l0, const glm::dvec2& b, double l1, const glm::dvec2& c, double l2, double r0, double r1, double r2, const glm::dvec2& prev_pos, const glm::dvec2& prev_pos2, const glm::dvec2& prev_pos3, double theta0, double theta1, double delta_theta);
	void circleFit(const std::vector<glm::dvec2>& p, glm::dvec2& c, double& r);
	glm::dvec2 closestPoint(const std::vector<glm::dvec2>& points, const glm::dvec2& p, int num_samples = 200);

	glm::dvec2 reflect(const glm::dvec2& p, const glm::dvec2& a, const glm::dvec2& v);
	glm::dmat3x3 affineTransform(const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& q1, const glm::dvec2& q2);
	double crossProduct(const glm::dvec2& v1, const glm::dvec2& v2);

	double area(const std::vector<glm::dvec2>& points);
	bool withinPolygon(const std::vector<glm::dvec2>& points, const glm::dvec2& pt);
	bool withinPolygon(const std::vector<std::vector<glm::dvec2>>& polygons, const glm::dvec2& pt);
	BBox boundingBox(const std::vector<glm::dvec2>& points);

	typedef std::vector<glm::dvec2> polygon;

}

BOOST_GEOMETRY_REGISTER_POINT_2D(glm::dvec2, double, boost::geometry::cs::cartesian, x, y)
BOOST_GEOMETRY_REGISTER_RING(kinematics::polygon)
