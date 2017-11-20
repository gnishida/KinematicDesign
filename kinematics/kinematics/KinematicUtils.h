#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/core/point_type.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include "BBox.h"

namespace kinematics {

	class Polygon25D {
	public:
		std::vector<glm::dvec2> points;
		std::vector<glm::dvec2> points2;    // if the top face has different points, use points2
		double depth1;
		double depth2;
		bool check_collision;

	public:
		Polygon25D() : depth1(0), depth2(0), check_collision(false) {}
		Polygon25D(const std::vector<glm::dvec2>& points, double depth1, double depth2, bool check_collision = true) : points(points), depth1(depth1), depth2(depth2), check_collision(check_collision) {}
		Polygon25D(const std::vector<glm::dvec2>& points, const std::vector<glm::dvec2>& points2, double depth1, double depth2, bool check_collision = true) : points(points), points2(points2), depth1(depth1), depth2(depth2), check_collision(check_collision) {}
	};

	class Object25D {
	public:
		std::vector<Polygon25D> polygons;
		Polygon25D operator[](int index) const {
			return polygons[index];
		}
		Polygon25D& operator[](int index) {
			return polygons[index];
		}
		void push_back(const Polygon25D& polygon) {
			polygons.push_back(polygon);
		}
		size_t size() const {
			return polygons.size();
		}
		Polygon25D& back() {
			return polygons.back();
		}
		Polygon25D back() const {
			return polygons.back();
		}

	public:
		Object25D() {}
		Object25D(const Polygon25D& polygon) {
			polygons.push_back(polygon);
		}
		Object25D(const std::vector<glm::dvec2>& points, double depth1, double depth2) {
			polygons.push_back(Polygon25D(points, depth1, depth2));
		}
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
	double distanceToSegment(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c);
	glm::dvec2 circleCenterFromThreePoints(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c);
	glm::dvec2 threeLengths(const glm::dvec2& a, double l0, const glm::dvec2& b, double l1, const glm::dvec2& c, double l2, double r0, double r1, double r2, const glm::dvec2& prev_pos, const glm::dvec2& prev_pos2, const glm::dvec2& prev_pos3);
	double threeLengths(const glm::dvec2& a, double l0, const glm::dvec2& b, double l1, const glm::dvec2& c, double l2, double r0, double r1, double r2, const glm::dvec2& prev_pos, const glm::dvec2& prev_pos2, const glm::dvec2& prev_pos3, double theta0, double theta1, double delta_theta);
	void circleFit(const std::vector<glm::dvec2>& p, glm::dvec2& c, double& r);
	glm::dvec2 closestOffsetPoint(const std::vector<glm::dvec2>& points, const glm::dvec2& p, double margin, int num_samples = 200);
	glm::dvec2 closestPoint(const std::vector<glm::dvec2>& points, const glm::dvec2& p, double& dist, int num_samples = 200);

	glm::dvec2 reflect(const glm::dvec2& p, const glm::dvec2& a, const glm::dvec2& v);
	glm::dmat3x3 affineTransform(const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& q1, const glm::dvec2& q2);
	double crossProduct(const glm::dvec2& v1, const glm::dvec2& v2);
	glm::dmat3x3 calculateTransMatrix(const glm::dvec2& O, const glm::dvec2& X);

	double area(const std::vector<glm::dvec2>& points);
	bool withinPolygon(const std::vector<glm::dvec2>& points, const glm::dvec2& pt);
	bool withinPolygon(const std::vector<std::vector<glm::dvec2>>& polygons, const glm::dvec2& pt);
	BBox boundingBox(const std::vector<glm::dvec2>& points);
	std::vector<std::vector<glm::dvec2>> unionPolygon(std::vector<std::vector<glm::dvec2>> polygons);

	std::vector<glm::dvec2> generateBarPolygon(const glm::dvec2& p1, const glm::dvec2& p2, float link_width);
	std::vector<glm::dvec2> generateRoundedBarPolygon(const glm::dvec2& p1, const glm::dvec2& p2, float link_radius, int num_slices = 36);
	std::vector<glm::dvec2> generateCirclePolygon(const glm::dvec2& p, float radius, int num_slices = 36);

	typedef std::vector<glm::dvec2> contour;
	typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> polygon;

}

BOOST_GEOMETRY_REGISTER_POINT_2D(glm::dvec2, double, boost::geometry::cs::cartesian, x, y)
BOOST_GEOMETRY_REGISTER_RING(kinematics::contour)
