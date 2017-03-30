#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/core/point_type.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>

namespace kinematics {

	const double M_PI = 3.14159265;
	const double TOL = 0.0000001;

	glm::dvec2 rotatePoint(const glm::dvec2& pt, double angle);
	glm::dvec2 rotatePoint(const glm::dvec2& pt, double angle, const glm::dvec2& rotation_center);
	glm::dvec2 circleCircleIntersection(const glm::dvec2& center1, double radius1, const glm::dvec2& center2, double radius);
	glm::dvec2 circleCircleIntersection(const glm::dvec2& center1, double radius1, const glm::dvec2& center2, double radius, const glm::dvec2& prev_int);
	glm::dvec2 circleLineIntersection(const glm::dvec2& center, double radius, const glm::dvec2& p1, const glm::dvec2& p2);
	glm::dvec2 circleLineIntersection(const glm::dvec2& center, double radius, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& prev_int);
	glm::dvec2 lineLineIntersection(const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const glm::dvec2& p4);
	bool polygonPolygonIntersection(const std::vector<glm::dvec2>& polygon1, const std::vector<glm::dvec2>& polygon2);
	bool pointWithinPolygon(const glm::dvec2& pt, const std::vector<glm::dvec2>& polygon);

	typedef std::vector<glm::dvec2> polygon;

}

BOOST_GEOMETRY_REGISTER_POINT_2D(glm::dvec2, double, boost::geometry::cs::cartesian, x, y)
BOOST_GEOMETRY_REGISTER_LINESTRING(kinematics::polygon)
