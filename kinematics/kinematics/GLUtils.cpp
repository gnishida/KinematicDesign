#include "GLUtils.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/ring.hpp>
#include <cassert>
#include <list>

namespace kinematics {
	namespace glutils {

		typedef boost::geometry::model::d2::point_xy<double>		point_2d;

		BoundingBox::BoundingBox() {
			minPt.x = std::numeric_limits<double>::max();
			minPt.y = std::numeric_limits<double>::max();
			minPt.z = std::numeric_limits<double>::max();
			maxPt.x = -std::numeric_limits<double>::max();
			maxPt.y = -std::numeric_limits<double>::max();
			maxPt.z = -std::numeric_limits<double>::max();
		}

		BoundingBox::BoundingBox(const std::vector<glm::dvec2>& points) {
			minPt.x = std::numeric_limits<double>::max();
			minPt.y = std::numeric_limits<double>::max();
			minPt.z = 0.0;
			maxPt.x = -std::numeric_limits<double>::max();
			maxPt.y = -std::numeric_limits<double>::max();
			maxPt.z = 0.0;

			for (int i = 0; i < points.size(); ++i) {
				minPt.x = std::min(minPt.x, points[i].x);
				minPt.y = std::min(minPt.y, points[i].y);
				maxPt.x = std::max(maxPt.x, points[i].x);
				maxPt.y = std::max(maxPt.y, points[i].y);
			}
		}

		BoundingBox::BoundingBox(const std::vector<glm::dvec3>& points) {
			minPt.x = std::numeric_limits<double>::max();
			minPt.y = std::numeric_limits<double>::max();
			minPt.z = std::numeric_limits<double>::max();
			maxPt.x = -std::numeric_limits<double>::max();
			maxPt.y = -std::numeric_limits<double>::max();
			maxPt.z = -std::numeric_limits<double>::max();

			for (int i = 0; i < points.size(); ++i) {
				minPt.x = std::min(minPt.x, points[i].x);
				minPt.y = std::min(minPt.y, points[i].y);
				minPt.z = std::min(minPt.z, points[i].z);
				maxPt.x = std::max(maxPt.x, points[i].x);
				maxPt.y = std::max(maxPt.y, points[i].y);
				maxPt.z = std::max(maxPt.z, points[i].z);
			}
		}

		BoundingBox::BoundingBox(const std::vector<std::vector<glm::dvec3> >& points) {
			minPt.x = (std::numeric_limits<double>::max)();
			minPt.y = (std::numeric_limits<double>::max)();
			minPt.z = (std::numeric_limits<double>::max)();
			maxPt.x = -(std::numeric_limits<double>::max)();
			maxPt.y = -(std::numeric_limits<double>::max)();
			maxPt.z = -(std::numeric_limits<double>::max)();

			for (int i = 0; i < points.size(); ++i) {
				for (int k = 0; k < points[i].size(); ++k) {
					minPt.x = std::min(minPt.x, points[i][k].x);
					minPt.y = std::min(minPt.y, points[i][k].y);
					minPt.z = std::min(minPt.z, points[i][k].z);
					maxPt.x = std::max(maxPt.x, points[i][k].x);
					maxPt.y = std::max(maxPt.y, points[i][k].y);
					maxPt.z = std::max(maxPt.z, points[i][k].z);
				}
			}
		}

		void BoundingBox::addPoint(const glm::dvec2& point) {
			minPt.x = std::min(minPt.x, point.x);
			minPt.y = std::min(minPt.y, point.y);
			maxPt.x = std::max(maxPt.x, point.x);
			maxPt.y = std::max(maxPt.y, point.y);
		}

		void BoundingBox::addPoint(const glm::dvec3& point) {
			minPt.x = std::min(minPt.x, point.x);
			minPt.y = std::min(minPt.y, point.y);
			minPt.z = std::min(minPt.z, point.z);
			maxPt.x = std::max(maxPt.x, point.x);
			maxPt.y = std::max(maxPt.y, point.y);
			maxPt.z = std::max(maxPt.z, point.z);
		}

		bool BoundingBox::contains(const glm::dvec2& point, double threshold) const {
			if (point.x < minPt.x - threshold || point.x > maxPt.x + threshold) return false;
			if (point.y < minPt.y - threshold || point.y > maxPt.y + threshold) return false;
			return true;
		}

		bool BoundingBox::contains(const glm::dvec3& point, double threshold) const {
			if (point.x < minPt.x - threshold || point.x > maxPt.x + threshold) return false;
			if (point.y < minPt.y - threshold || point.y > maxPt.y + threshold) return false;
			if (point.z < minPt.z - threshold || point.z > maxPt.z + threshold) return false;
			return true;
		}

		float croosProduct(const glm::vec2& a, const glm::vec2& b) {
			return a.x * b.y - a.y - b.x;
		}

		/**
		 * Test if the point is inside the polygon
		 */
		bool isWithinPolygon(const glm::vec2& p, const std::vector<glm::vec2>& points) {
			boost::geometry::model::ring<point_2d> contour;
			for (int i = 0; i < points.size(); ++i) {
				contour.push_back(point_2d(points[i].x, points[i].y));
			}
			boost::geometry::correct(contour);

			return boost::geometry::within(point_2d(p.x, p.y), contour);
		}

		bool isWithinPolygon(const glm::dvec2& p, const std::vector<glm::dvec2>& points) {
			boost::geometry::model::ring<point_2d> contour;
			for (int i = 0; i < points.size(); ++i) {
				contour.push_back(point_2d(points[i].x, points[i].y));
			}
			boost::geometry::correct(contour);

			return boost::geometry::within(point_2d(p.x, p.y), contour);
		}

		float area(const std::vector<glm::vec2>& points) {
			boost::geometry::model::ring<point_2d> contour;
			for (int i = 0; i < points.size(); ++i) {
				contour.push_back(point_2d(points[i].x, points[i].y));
			}
			boost::geometry::correct(contour);

			return boost::geometry::area(contour);
		}

		/**
			* Compute the offset polygon.
			*/
		void offsetPolygon(const std::vector<glm::vec2>& points, float offsetDistance, std::vector<glm::vec2>& offset_points) {
			offset_points.clear();

			Polygon_2 poly;

			for (int i = 0; i < points.size(); ++i) {
				poly.push_back(K::Point_2(points[i].x, points[i].y));
			}

			std::vector<PolygonPtr> offset_poly;
			if (offsetDistance >= 0) {
				K::FT lOffset = offsetDistance;
				offset_poly = CGAL::create_exterior_skeleton_and_offset_polygons_2(lOffset, poly);

				for (auto it = offset_poly[1]->vertices_begin(); it != offset_poly[1]->vertices_end(); ++it) {
					offset_points.push_back(glm::vec2(it->x(), it->y()));
				}
				std::reverse(offset_points.begin(), offset_points.end());
			}
			else {
				K::FT lOffset = -offsetDistance;
				offset_poly = CGAL::create_interior_skeleton_and_offset_polygons_2(lOffset, poly);

				for (auto it = offset_poly[0]->vertices_begin(); it != offset_poly[0]->vertices_end(); ++it) {
					offset_points.push_back(glm::vec2(it->x(), it->y()));
				}
			}
		}

		/*
			* Return the distance from segment ab to point c.
			*/
		float distance(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c, bool segmentOnly) {
			float r_numerator = (c.x-a.x) * (b.x-a.x) + (c.y-a.y) * (b.y-a.y);
			float r_denomenator = (b.x-a.x) * (b.x-a.x) + (b.y-a.y) * (b.y-a.y);

			if (r_denomenator <= 0.0f) {
				return (a - c).length();
			}

			float r = r_numerator / r_denomenator;

			if (segmentOnly && (r < 0 || r > 1)) {
				float dist1 = std::hypot(c.x - a.x, c.y - a.y);
				//float dist1 = SQR(c.x - a.x) + SQR(c.y - a.y);
				float dist2 = std::hypot(c.x - b.x, c.y - b.y);
				//float dist2 = SQR(c.x - b.x) + SQR(c.y - b.y);
				if (dist1 < dist2) {	
					return dist1;
					//return sqrt(dist1);
				} else {
					return dist2;
					//return sqrt(dist2);
				}
			} else {
				return abs((a.y-c.y) * (b.x-a.x) - (a.x-c.x) * (b.y-a.y)) / sqrt(r_denomenator);
			}
		}

		/*
			* Return the distance from segment ab to point c.
			*/
		float distance(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
			glm::vec3 v = glm::normalize(b - a);
			return glm::length(glm::cross(a - c, v));
		}

		/**
			* Line-line intersection
			* Compute the intersection of a line that pass though p1 and whose vector is v1 and another line that pass through p2 and whose vector is v2.
			*/
		glm::vec3 lineLineIntersection(const glm::vec3& p1, const glm::vec3& v1, const glm::vec3& p2, const glm::vec3& v2, float weight1, float weight2) {
			// tentative implementation (might be wrong)
			glm::mat2 m1;
			m1[0].x = glm::dot(v1, v1);
			m1[0].y = glm::dot(v1, v2);
			m1[1].x = glm::dot(v1, -v2);
			m1[1].y = glm::dot(v2, -v2);

			glm::vec2 m2;
			m2.x = glm::dot(v1, p2 - p1);
			m2.y = glm::dot(v2, p2 - p1);

			glm::vec2 st = glm::inverse(m1) * m2;

			glm::vec3 pp1 = p1 + v1 * st.x;
			glm::vec3 pp2 = p2 + v2 * st.y;

			return (pp1 * weight1 + pp2 * weight2) / (weight1 + weight2);
		}

		/**
			* Ray-Plane intersection
			* Compute the intersection of a ray that starts from a and its direction v, and a plane whose normal is n and p is on the plane.
			*/
		glm::vec3 rayPlaneIntersection(const glm::vec3& a, const glm::vec3& v, const glm::vec3& p, const glm::vec3& n) {
			return a + v * glm::dot(p - a, n) / glm::dot(v, n);
		}

		/**
			* Ray-Triangle intersection
			*/
		bool rayTriangleIntersection(const glm::vec3& a, const glm::vec3& v, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, glm::vec3& intPt) {
			glm::vec3 n = glm::cross(p2 - p1, p3 - p1);
			intPt = a + v * glm::dot(p1 - a, n) / glm::dot(v, n);

			cv::Mat_<double> L(3, 1);
			L(0, 0) = (intPt - p1).x;
			L(1, 0) = (intPt - p1).y;
			L(2, 0) = (intPt - p1).z;
			cv::Mat_<double> R(3, 2);
			R(0, 0) = (p2 - p1).x;
			R(1, 0) = (p2 - p1).y;
			R(2, 0) = (p2 - p1).z;
			R(0, 1) = (p3 - p1).x;
			R(1, 1) = (p3 - p1).y;
			R(2, 1) = (p3 - p1).z;

			cv::Mat_<double> st = R.inv(cv::DECOMP_SVD) * L;
		
			if (st(0, 0) >= 0 && st(1, 0) >= 0 && st(0, 0) + st(1, 0) <= 1) return true;
			else return false;
		}

		/**
			* Compute the barycentroic coordinates.
			*
			* @param p1	point 1
			* @param p2	point 2
			* @param p3	point 3
			* @param p		point
			* @return		the barycentroic coodinates
			*/
		glm::vec2 barycentricCoordinates(const glm::vec2& p1, const glm::vec2& p2, const glm::vec2& p3, const glm::vec2& p) {
			float den = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
			float alpha = ((p.x - p1.x) * (p3.y - p1.y) - (p.y - p1.y) * (p3.x - p1.x)) / den;
			float beta = ((p.y - p1.y) * (p2.x - p1.x) - (p.x - p1.x) * (p2.y - p1.y)) / den;

			return glm::vec2(alpha, beta);
		}

		void drawCircle(float r1, float r2, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices, int slices) {
			glm::vec3 p1(mat * glm::vec4(0, 0, 0, 1));
			glm::vec3 n(mat * glm::vec4(0, 0, 1, 0));

			for (int i = 0; i < slices; ++i) {
				float theta1 = (float)i / slices * M_PI * 2.0f;
				float theta2 = (float)(i + 1) / slices * M_PI * 2.0f;

				glm::vec4 p2(cosf(theta1) * r1, sinf(theta1) * r2, 0, 1);
				glm::vec4 p3(cosf(theta2) * r1, sinf(theta2) * r2, 0, 1);

				p2 = mat * p2;
				p3 = mat * p3;
			
				vertices.push_back(Vertex(glm::vec3(p1), glm::vec3(n), color));
				vertices.push_back(Vertex(glm::vec3(p2), glm::vec3(n), color, 1));
				vertices.push_back(Vertex(glm::vec3(p3), glm::vec3(n), color, 1));
			}
		}

		void drawCircle(float r1, float r2, float texWidth, float texHeight, const glm::mat4& mat, std::vector<Vertex>& vertices, int slices) {
			glm::vec3 p1(mat * glm::vec4(0, 0, 0, 1));
			glm::vec3 n(mat * glm::vec4(0, 0, 1, 0));
			glm::vec2 t1(r1 / texWidth, r2 / texHeight);

			for (int i = 0; i < slices; ++i) {
				float theta1 = (float)i / slices * M_PI * 2.0f;
				float theta2 = (float)(i + 1) / slices * M_PI * 2.0f;

				glm::vec4 p2(cosf(theta1) * r1, sinf(theta1) * r2, 0, 1);
				glm::vec4 p3(cosf(theta2) * r1, sinf(theta2) * r2, 0, 1);

				glm::vec2 t2((p2.x + r1) / texWidth, (p2.y + r2) / texHeight);
				glm::vec2 t3((p3.x + r1) / texWidth, (p3.y + r2) / texHeight);

				p2 = mat * p2;
				p3 = mat * p3;

				vertices.push_back(Vertex(p1, n, glm::vec4(1, 1, 1, 1), t1));
				vertices.push_back(Vertex(glm::vec3(p2), n, glm::vec4(1, 1, 1, 1), t2, 1));
				vertices.push_back(Vertex(glm::vec3(p3), n, glm::vec4(1, 1, 1, 1), t3, 1));
			}
		}

		void drawQuad(float w, float h, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices) {
			glm::vec3 p1(mat * glm::vec4(-w * 0.5, -h * 0.5, 0, 1));
			glm::vec3 p2(mat * glm::vec4(w * 0.5, -h * 0.5, 0, 1));
			glm::vec3 p3(mat * glm::vec4(w * 0.5, h * 0.5, 0, 1));
			glm::vec3 p4(mat * glm::vec4(-w * 0.5, h * 0.5, 0, 1));
			glm::vec3 n(mat * glm::vec4(0, 0, 1, 0));

			vertices.push_back(Vertex(p1, n, color, glm::vec2(0, 0)));
			vertices.push_back(Vertex(p2, n, color, glm::vec2(1, 0), 1));
			vertices.push_back(Vertex(p3, n, color, glm::vec2(1, 1)));

			vertices.push_back(Vertex(p1, n, color, glm::vec2(0, 0)));
			vertices.push_back(Vertex(p3, n, color, glm::vec2(1, 1)));
			vertices.push_back(Vertex(p4, n, color, glm::vec2(0, 1), 1));
		}

		void drawQuad(float w, float h, const glm::vec2& t1, const glm::vec2& t2, const glm::vec2& t3, const glm::vec2& t4, const glm::mat4& mat, std::vector<Vertex>& vertices) {
			glm::vec3 p1(mat * glm::vec4(-w * 0.5, -h * 0.5, 0, 1));
			glm::vec3 p2(mat * glm::vec4(w * 0.5, -h * 0.5, 0, 1));
			glm::vec3 p3(mat * glm::vec4(w * 0.5, h * 0.5, 0, 1));
			glm::vec3 p4(mat * glm::vec4(-w * 0.5, h * 0.5, 0, 1));
			glm::vec3 n(mat * glm::vec4(0, 0, 1, 0));

			vertices.push_back(Vertex(p1, n, glm::vec4(1, 1, 1, 1), t1));
			vertices.push_back(Vertex(p2, n, glm::vec4(1, 1, 1, 1), t2, 1));
			vertices.push_back(Vertex(p3, n, glm::vec4(1, 1, 1, 1), t3));

			vertices.push_back(Vertex(p1, n, glm::vec4(1, 1, 1, 1), t1));
			vertices.push_back(Vertex(p3, n, glm::vec4(1, 1, 1, 1), t3));
			vertices.push_back(Vertex(p4, n, glm::vec4(1, 1, 1, 1), t4, 1));
		}

		void drawPolygon(const std::vector<glm::vec3>& points, const glm::vec4& color, const std::vector<glm::vec2>& texCoords, const glm::mat4& mat, std::vector<Vertex>& vertices) {
			glm::vec3 p1(mat * glm::vec4(points.back(), 1));
			glm::vec2 t1 = texCoords.back();
			glm::vec3 p2(mat * glm::vec4(points[0], 1));
			glm::vec2 t2 = texCoords[0];

			// calculate the normal vector
			glm::vec3 normal(mat * glm::vec4(0, 0, 1, 0));

			for (int i = 0; i < points.size() - 2; ++i) {
				glm::vec3 p3(mat * glm::vec4(points[i + 1], 1));
				glm::vec2 t3 = texCoords[i + 1];

				// create the triangle only if two adjacent edges are not collinear.
				if (glm::length(glm::cross(p2 - p1, p3 - p2)) > 0.01) {
					vertices.push_back(Vertex(glm::vec3(p1), normal, color, t1));
					if (i < points.size() - 3) {
						vertices.push_back(Vertex(glm::vec3(p2), normal, color, t2, 1));
					}
					else {
						vertices.push_back(Vertex(glm::vec3(p2), normal, color, t2));
					}
					if (i > 0) {
						vertices.push_back(Vertex(glm::vec3(p3), normal, color, t3, 1));
					}
					else {
						vertices.push_back(Vertex(glm::vec3(p3), normal, color, t3));
					}
				}

				p2 = p3;
				t2 = t3;
			}
		}

		void drawPolygon(const std::vector<glm::vec3>& points, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices) {
			glm::vec3 p1(mat * glm::vec4(points.back(), 1));
			glm::vec3 p2(mat * glm::vec4(points[0], 1));

			// calculate the normal vector
			glm::vec3 normal(mat * glm::vec4(0, 0, 1, 0));

			for (int i = 0; i < points.size() - 2; ++i) {
				glm::vec3 p3(mat * glm::vec4(points[i + 1], 1));

				// create the triangle only if two adjacent edges are not collinear.
				if (glm::length(glm::cross(p2 - p1, p3 - p2)) > 0.01) {
					vertices.push_back(Vertex(glm::vec3(p1), normal, color));
					if (i < points.size() - 3) {
						vertices.push_back(Vertex(glm::vec3(p2), normal, color, 1));
					}
					else {
						vertices.push_back(Vertex(glm::vec3(p2), normal, color));
					}
					if (i > 0) {
						vertices.push_back(Vertex(glm::vec3(p3), normal, color, 1));
					}
					else {
						vertices.push_back(Vertex(glm::vec3(p3), normal, color));
					}
				}

				p2 = p3;
			}
		}

		void drawPolygon(const std::vector<glm::vec2>& points, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices, bool flip) {
			glm::vec3 p1(mat * glm::vec4(points.back(), 0, 1));
			glm::vec3 p2(mat * glm::vec4(points[0], 0, 1));

			// calculate the normal vector
			glm::vec3 normal(mat * glm::vec4(0, 0, 1, 0));

			for (int i = 0; i < points.size() - 2; ++i) {
				glm::vec3 p3(mat * glm::vec4(points[i + 1], 0, 1));

				// create the triangle only if two adjacent edges are not collinear.
				if (glm::length(glm::cross(p2 - p1, p3 - p2)) > 0.01) {
					vertices.push_back(Vertex(glm::vec3(p1), normal, color));
					if (i < points.size() - 3) {
						vertices.push_back(Vertex(glm::vec3(p2), normal, color, 1));
					}
					else {
						vertices.push_back(Vertex(glm::vec3(p2), normal, color));
					}
					if (i > 0) {
						vertices.push_back(Vertex(glm::vec3(p3), normal, color, 1));
					}
					else {
						vertices.push_back(Vertex(glm::vec3(p3), normal, color));
					}
				}

				p2 = p3;
			}
		}

		void drawPolygon(const std::vector<glm::vec2>& points, const glm::vec4& color, const std::vector<glm::vec2>& texCoords, const glm::mat4& mat, std::vector<Vertex>& vertices, bool flip) {
			glm::vec3 p1(mat * glm::vec4(points.back(), 0, 1));
			glm::vec2 t1 = texCoords.back();
			glm::vec3 p2(mat * glm::vec4(points[0], 0, 1));
			glm::vec2 t2 = texCoords[0];

			// calculate the normal vector
			glm::vec3 normal(mat * glm::vec4(0, 0, 1, 0));

			for (int i = 0; i < points.size() - 2; ++i) {
				glm::vec3 p3(mat * glm::vec4(points[i + 1], 0, 1));
				glm::vec2 t3 = texCoords[i + 1];

				// create the triangle only if two adjacent edges are not collinear.
				if (glm::length(glm::cross(p2 - p1, p3 - p2)) > 0.01) {
					vertices.push_back(Vertex(glm::vec3(p1), normal, color, t1));
					if (i < points.size() - 3) {
						vertices.push_back(Vertex(glm::vec3(p2), normal, color, t2, 1));
					}
					else {
						vertices.push_back(Vertex(glm::vec3(p2), normal, color, t2));
					}
					if (i > 0) {
						vertices.push_back(Vertex(glm::vec3(p3), normal, color, t3, 1));
					}
					else {
						vertices.push_back(Vertex(glm::vec3(p3), normal, color, t3));
					}
				}

				p2 = p3;
				t2 = t3;
			}
		}

		void drawConcavePolygon(const std::vector<glm::vec2>& points, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices, bool flip) {
			float min_x = std::numeric_limits<float>::max();
			float max_x = -std::numeric_limits<float>::max();
			float min_y = std::numeric_limits<float>::max();
			float max_y = -std::numeric_limits<float>::max();

			Polygon_2 polygon;
			for (int i = 0; i < points.size(); ++i) {
				polygon.push_back(Point_2(points[i].x, points[i].y));

				min_x = std::min(min_x, points[i].x);
				max_x = std::max(max_x, points[i].x);
				min_y = std::min(min_y, points[i].y);
				max_y = std::max(max_y, points[i].y);
			}

			if (polygon.is_clockwise_oriented()) {
				polygon.reverse_orientation();
			}
		
			// tesselate the concave polygon
			Polygon_list partition_polys;
			Traits       partition_traits;
			CGAL::greene_approx_convex_partition_2(polygon.vertices_begin(), polygon.vertices_end(), std::back_inserter(partition_polys), partition_traits);

			for (auto fit = partition_polys.begin(); fit != partition_polys.end(); ++fit) {
				std::vector<glm::vec2> pts;
				std::vector<glm::vec2> texCoords;
				for (auto vit = fit->vertices_begin(); vit != fit->vertices_end(); ++vit) {
					pts.push_back(glm::vec2(vit->x(), vit->y()));
					texCoords.push_back(glm::vec2((vit->x() - min_x) / (max_x - min_x), (vit->y() - min_y) / (max_y - min_y)));
				}

				drawPolygon(pts, color, texCoords, mat, vertices, flip);
			}
		}

		void drawConcavePolygon(const std::vector<glm::dvec2>& points, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices, bool flip) {
			double min_x = std::numeric_limits<double>::max();
			double max_x = -std::numeric_limits<double>::max();
			double min_y = std::numeric_limits<double>::max();
			double max_y = -std::numeric_limits<double>::max();

			Polygon_2 polygon;
			for (int i = 0; i < points.size(); ++i) {
				polygon.push_back(Point_2(points[i].x, points[i].y));

				min_x = std::min(min_x, points[i].x);
				max_x = std::max(max_x, points[i].x);
				min_y = std::min(min_y, points[i].y);
				max_y = std::max(max_y, points[i].y);
			}

			if (polygon.is_clockwise_oriented()) {
				polygon.reverse_orientation();
			}

			// tesselate the concave polygon
			Polygon_list partition_polys;
			Traits       partition_traits;
			CGAL::greene_approx_convex_partition_2(polygon.vertices_begin(), polygon.vertices_end(), std::back_inserter(partition_polys), partition_traits);

			for (auto fit = partition_polys.begin(); fit != partition_polys.end(); ++fit) {
				std::vector<glm::vec2> pts;
				std::vector<glm::vec2> texCoords;
				for (auto vit = fit->vertices_begin(); vit != fit->vertices_end(); ++vit) {
					pts.push_back(glm::vec2(vit->x(), vit->y()));
					texCoords.push_back(glm::vec2((vit->x() - min_x) / (max_x - min_x), (vit->y() - min_y) / (max_y - min_y)));
				}

				drawPolygon(pts, color, texCoords, mat, vertices, flip);
			}
		}

		void mark_domains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border) {
			if (start->info().nesting_level != -1){
				return;
			}
			std::list<CDT::Face_handle> queue;
			queue.push_back(start);
			while (!queue.empty()){
				CDT::Face_handle fh = queue.front();
				queue.pop_front();
				if (fh->info().nesting_level == -1){
					fh->info().nesting_level = index;
					for (int i = 0; i < 3; i++){
						CDT::Edge e(fh, i);
						CDT::Face_handle n = fh->neighbor(i);
						if (n->info().nesting_level == -1){
							if (ct.is_constrained(e)) {
								border.push_back(e);


							}
							else queue.push_back(n);
						}
					}
				}
			}
		}

		//explore set of facets connected with non constrained edges,
		//and attribute to each such set a nesting level.
		//We start from facets incident to the infinite vertex, with a nesting
		//level of 0. Then we recursively consider the non-explored facets incident 
		//to constrained edges bounding the former set and increase the nesting level by 1.
		//Facets in the domain are those with an odd nesting level.
		void mark_domains(CDT& cdt) {
			for (CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it){
				it->info().nesting_level = -1;
			}
			std::list<CDT::Edge> border;
			mark_domains(cdt, cdt.infinite_face(), 0, border);
			while (!border.empty()){
				CDT::Edge e = border.front();
				border.pop_front();
				CDT::Face_handle n = e.first->neighbor(e.second);
				if (n->info().nesting_level == -1){
					mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
				}
			}
		}

		void drawConcavePolygon(const std::vector<glm::dvec2>& points, const std::vector<std::vector<glm::dvec2>>& holes, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices, bool flip) {
			// Calculate the normal vector
			glm::vec3 n(mat * glm::vec4(0, 0, 1, 0));

			//Insert the polygons into a constrained triangulation
			CDT cdt;
			Polygon_2 polygon;
			for (int j = 0; j < points.size(); j++) {
				polygon.push_back(Point(points[j].x, points[j].y));
			}
			cdt.insert_constraint(polygon.vertices_begin(), polygon.vertices_end(), true);
			for (int i = 0; i < holes.size(); i++) {
				Polygon_2 polygon;
				for (int j = 0; j < holes[i].size(); j++) {
					polygon.push_back(Point(holes[i][j].x, holes[i][j].y));
				}
				cdt.insert_constraint(polygon.vertices_begin(), polygon.vertices_end(), true);
			}

			//Mark facets that are inside the domain bounded by the polygon
			mark_domains(cdt);

			for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); ++fit) {
				if (fit->info().in_domain()) {
					std::vector<glm::vec2> triangle;
					for (int i = 0; i < 3; i++) {
						CDT::Vertex_handle vh = fit->vertex(i);
						triangle.push_back(glm::vec2(vh->point().x(), vh->point().y()));
					}
					drawPolygon(triangle, color, mat, vertices, flip);
				}
			}
		}

		void drawConcavePolygon(const std::vector<glm::vec2>& points, const glm::vec4& color, const std::vector<glm::vec2>& texCoords, const glm::mat4& mat, std::vector<Vertex>& vertices, bool flip) {
			Polygon_2 polygon;
			for (int i = 0; i < points.size(); ++i) {
				polygon.push_back(Point_2(points[i].x, points[i].y));
			}

			if (polygon.is_clockwise_oriented()) {
				polygon.reverse_orientation();
			}
			
			// tesselate the concave polygon
			Polygon_list partition_polys;
			Traits       partition_traits;
			CGAL::greene_approx_convex_partition_2(polygon.vertices_begin(), polygon.vertices_end(), std::back_inserter(partition_polys), partition_traits);

			for (auto fit = partition_polys.begin(); fit != partition_polys.end(); ++fit) {
				std::vector<glm::vec2> pts;
				std::vector<glm::vec2> tex;

				for (auto vit = fit->vertices_begin(); vit != fit->vertices_end(); ++vit) {
					glm::vec2 pt(vit->x(), vit->y());
					pts.push_back(pt);

					// find the same point to get texture coordinates
					bool found = false;
					glm::vec2 texCoord;
					for (int i = 0; i < points.size(); ++i) {
						if (glm::length(points[i] - pt) < 0.01f) {
							found = true;
							texCoord = texCoords[i];
						}
					}

					// if not, estimate the texture coordinates based on the three neighbors
					if (!found) {
						glm::vec2 p1 = points[0];
						glm::vec2 t1 = texCoords[0];
						glm::vec2 p2 = points[1];
						glm::vec2 t2 = texCoords[1];
						glm::vec2 p3 = points[2];
						glm::vec2 t3 = texCoords[2];
						if (glm::dot(glm::normalize(p2 - p1), glm::normalize(p3 - p2)) > 0.99f && points.size() > 3 && texCoords.size() > 3) {
							p3 = points[3];
							t3 = texCoords[3];
						}

						glm::vec2 bc = barycentricCoordinates(p1, p2, p3, pt);
						texCoord = t1 * (1.0f - bc.x - bc.y) + t2 * bc.x + t3 * bc.y;
					}

					tex.push_back(texCoord);
				}

				drawPolygon(pts, color, tex, mat, vertices, flip);
			}
		}

		void drawGrid(float width, float height, float cell_size, const glm::vec4& lineColor, const glm::vec4& backgroundColor, const glm::mat4& mat, std::vector<Vertex>& vertices) {
			drawQuad(width, height, backgroundColor, mat, vertices);

			float line_width = cell_size * 0.03;
		
			for (float x = 0; x < width * 0.5; x += cell_size) {
				if (x == 0) {
					glm::mat4 m = glm::translate(mat, glm::vec3(x, -height * 0.5, 0));
					drawCylinderY(line_width * 2, line_width * 2, height, lineColor, m, vertices);
				} else {
					glm::mat4 m = glm::translate(mat, glm::vec3(x, -height * 0.5, 0));
					drawCylinderY(line_width * 0.5, line_width * 0.5, height, lineColor, m, vertices);

					m = glm::translate(mat, glm::vec3(-x, -height * 0.5, 0));
					drawCylinderY(line_width * 0.5, line_width * 0.5, height, lineColor, m, vertices);
				}
			}

			for (float y = 0; y < height * 0.5; y += cell_size) {
				if (y == 0) {
					glm::mat4 m = glm::translate(mat, glm::vec3(-width * 0.5, y, 0));
					drawCylinderX(line_width * 2, line_width * 2, width, lineColor, m, vertices);
				} else {
					glm::mat4 m = glm::translate(mat, glm::vec3(-width * 0.5, y, 0));
					drawCylinderX(line_width * 0.5, line_width * 0.5, width, lineColor, m, vertices);

					m = glm::translate(mat, glm::vec3(-width * 0.5, -y, 0));
					drawCylinderX(line_width * 0.5, line_width * 0.5, width, lineColor, m, vertices);
				}
			}
		}

		void drawBox(float length_x, float length_y, float length_z, glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices) {
			glm::vec3 p1(mat * glm::vec4(-length_x * 0.5, -length_y * 0.5, -length_z * 0.5, 1));
			glm::vec3 p2(mat * glm::vec4(length_x * 0.5, -length_y * 0.5, -length_z * 0.5, 1));
			glm::vec3 p3(mat * glm::vec4(length_x * 0.5, length_y * 0.5, -length_z * 0.5, 1));
			glm::vec3 p4(mat * glm::vec4(-length_x * 0.5, length_y * 0.5, -length_z * 0.5, 1));
			glm::vec3 p5(mat * glm::vec4(-length_x * 0.5, -length_y * 0.5, length_z * 0.5, 1));
			glm::vec3 p6(mat * glm::vec4(length_x * 0.5, -length_y * 0.5, length_z * 0.5, 1));
			glm::vec3 p7(mat * glm::vec4(length_x * 0.5, length_y * 0.5, length_z * 0.5, 1));
			glm::vec3 p8(mat * glm::vec4(-length_x * 0.5, length_y * 0.5, length_z * 0.5, 1));
			glm::vec3 n1(mat * glm::vec4(-1, 0, 0, 0));
			glm::vec3 n2(mat * glm::vec4(1, 0, 0, 0));
			glm::vec3 n3(mat * glm::vec4(0, -1, 0, 0));
			glm::vec3 n4(mat * glm::vec4(0, 1, 0, 0));
			glm::vec3 n5(mat * glm::vec4(0, 0, -1, 0));
			glm::vec3 n6(mat * glm::vec4(0, 0, 1, 0));

			vertices.push_back(Vertex(p1, n5, color));
			vertices.push_back(Vertex(p4, n5, color, 1));
			vertices.push_back(Vertex(p3, n5, color));

			vertices.push_back(Vertex(p1, n5, color));
			vertices.push_back(Vertex(p3, n5, color));
			vertices.push_back(Vertex(p2, n5, color, 1));

			vertices.push_back(Vertex(p1, n3, color));
			vertices.push_back(Vertex(p2, n3, color, 1));
			vertices.push_back(Vertex(p6, n3, color));

			vertices.push_back(Vertex(p1, n3, color));
			vertices.push_back(Vertex(p6, n3, color));
			vertices.push_back(Vertex(p5, n3, color, 1));

			vertices.push_back(Vertex(p2, n2, color));
			vertices.push_back(Vertex(p3, n2, color, 1));
			vertices.push_back(Vertex(p7, n2, color));

			vertices.push_back(Vertex(p2, n2, color));
			vertices.push_back(Vertex(p7, n2, color));
			vertices.push_back(Vertex(p6, n2, color, 1));

			vertices.push_back(Vertex(p3, n4, color));
			vertices.push_back(Vertex(p4, n4, color, 1));
			vertices.push_back(Vertex(p8, n4, color));

			vertices.push_back(Vertex(p3, n4, color));
			vertices.push_back(Vertex(p8, n4, color));
			vertices.push_back(Vertex(p7, n4, color, 1));

			vertices.push_back(Vertex(p4, n1, color));
			vertices.push_back(Vertex(p1, n1, color, 1));
			vertices.push_back(Vertex(p5, n1, color));

			vertices.push_back(Vertex(p4, n1, color));
			vertices.push_back(Vertex(p5, n1, color));
			vertices.push_back(Vertex(p8, n1, color, 1));

			vertices.push_back(Vertex(p5, n6, color));
			vertices.push_back(Vertex(p6, n6, color, 1));
			vertices.push_back(Vertex(p7, n6, color));

			vertices.push_back(Vertex(p5, n6, color));
			vertices.push_back(Vertex(p7, n6, color));
			vertices.push_back(Vertex(p8, n6, color, 1));
		}

		void drawSphere(float radius, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices) {
			int slices = 12;
			int stacks = 6;

			for (int i = 0; i < stacks; ++i) {
				float phi1 = M_PI * (float)i / stacks - M_PI * 0.5;
				float phi2 = M_PI * (float)(i + 1) / stacks - M_PI * 0.5;
				float r1 = cosf(phi1) * radius;
				float r2 = cosf(phi2) * radius;

				for (int j = 0; j < slices; ++j) {
					float theta1 = M_PI * 2.0 * (float)j / slices;
					float theta2 = M_PI * 2.0 * (float)(j + 1) / slices;

					glm::vec3 p1(mat * glm::vec4(cosf(theta1) * r1, sinf(theta1) * r1, sinf(phi1) * radius, 1));
					glm::vec3 p2(mat * glm::vec4(cosf(theta2) * r1, sinf(theta2) * r1, sinf(phi1) * radius, 1));
					glm::vec3 p3(mat * glm::vec4(cosf(theta2) * r2, sinf(theta2) * r2, sinf(phi2) * radius, 1));
					glm::vec3 p4(mat * glm::vec4(cosf(theta1) * r2, sinf(theta1) * r2, sinf(phi2) * radius, 1));

					glm::vec3 n1(mat * glm::vec4(cosf(phi1) * cosf(theta1), cosf(phi1) * sinf(theta1), sinf(phi1), 0));
					glm::vec3 n2(mat * glm::vec4(cosf(phi1) * cosf(theta2), cosf(phi1) * sinf(theta2), sinf(phi1), 0));
					glm::vec3 n3(mat * glm::vec4(cosf(phi2) * cosf(theta2), cosf(phi2) * sinf(theta2), sinf(phi2), 0));
					glm::vec3 n4(mat * glm::vec4(cosf(phi2) * cosf(theta1), cosf(phi2) * sinf(theta1), sinf(phi2), 0));

					vertices.push_back(Vertex(p1, n1, color));
					vertices.push_back(Vertex(p2, n2, color, 1));
					vertices.push_back(Vertex(p3, n3, color));

					vertices.push_back(Vertex(p1, n1, color));
					vertices.push_back(Vertex(p3, n3, color));
					vertices.push_back(Vertex(p4, n4, color, 1));
				}
			}
		}

		void drawEllipsoid(float r1, float r2, float r3, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices) {
			int slices = 32;
			int stacks = 16;

			for (int i = 0; i < stacks; ++i) {
				float phi1 = M_PI * (float)i / stacks - M_PI * 0.5;
				float phi2 = M_PI * (float)(i + 1) / stacks - M_PI * 0.5;

				for (int j = 0; j < slices; ++j) {
					float theta1 = M_PI * 2.0 * (float)j / slices;
					float theta2 = M_PI * 2.0 * (float)(j + 1) / slices;

					glm::vec3 p1(mat * glm::vec4(cosf(phi1) * cosf(theta1) * r1, cosf(phi1) * sinf(theta1) * r2, sinf(phi1) * r3, 1));
					glm::vec3 p2(mat * glm::vec4(cosf(phi1) * cosf(theta2) * r1, cosf(phi1) * sinf(theta2) * r2, sinf(phi1) * r3, 1));
					glm::vec3 p3(mat * glm::vec4(cosf(phi2) * cosf(theta2) * r1, cosf(phi2) * sinf(theta2) * r2, sinf(phi2) * r3, 1));
					glm::vec3 p4(mat * glm::vec4(cosf(phi2) * cosf(theta1) * r1, cosf(phi2) * sinf(theta1) * r2, sinf(phi2) * r3, 1));

					glm::vec3 n1(mat * glm::vec4(cosf(phi1) * cosf(theta1), cosf(phi1) * sinf(theta1), sinf(phi1), 0));
					glm::vec3 n2(mat * glm::vec4(cosf(phi1) * cosf(theta2), cosf(phi1) * sinf(theta2), sinf(phi1), 0));
					glm::vec3 n3(mat * glm::vec4(cosf(phi2) * cosf(theta2), cosf(phi2) * sinf(theta2), sinf(phi2), 0));
					glm::vec3 n4(mat * glm::vec4(cosf(phi2) * cosf(theta1), cosf(phi2) * sinf(theta1), sinf(phi2), 0));

					vertices.push_back(Vertex(p1, n1, color));
					vertices.push_back(Vertex(p2, n2, color, 1));
					vertices.push_back(Vertex(p3, n3, color));

					vertices.push_back(Vertex(p1, n1, color));
					vertices.push_back(Vertex(p3, n3, color));
					vertices.push_back(Vertex(p4, n4, color, 1));
				}
			}
		}

		/**
		 * X軸方向に高さ h、底面の半径 r1、上面の半径 r2の円錐を描画する。
		 */
		void drawCylinderX(float radius1, float radius2, float h, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices, int slices) {
			float phi = atan2(radius1 - radius2, h);

			for (int i = 0; i < slices; ++i) {
				float theta1 = M_PI * 2.0 * (float)i / slices;
				float theta2 = M_PI * 2.0 * (float)(i + 1) / slices;

				glm::vec3 p1(mat * glm::vec4(0, cosf(theta1) * radius1, sinf(theta1) * radius1, 1));
				glm::vec3 p2(mat * glm::vec4(0, cosf(theta2) * radius1, sinf(theta2) * radius1, 1));
				glm::vec3 p3(mat * glm::vec4(h, cosf(theta2) * radius2, sinf(theta2) * radius2, 1));
				glm::vec3 p4(mat * glm::vec4(h, cosf(theta1) * radius2, sinf(theta1) * radius2, 1));
				glm::vec3 n1(mat * glm::vec4(sinf(phi), cosf(theta1) * cosf(phi), sinf(theta1) * cosf(phi), 0));
				glm::vec3 n2(mat * glm::vec4(sinf(phi), cosf(theta2) * cosf(phi), sinf(theta2) * cosf(phi), 0));

				vertices.push_back(Vertex(p1, n1, color));
				vertices.push_back(Vertex(p2, n2, color, 1));
				vertices.push_back(Vertex(p3, n2, color));

				vertices.push_back(Vertex(p1, n1, color));
				vertices.push_back(Vertex(p3, n2, color));
				vertices.push_back(Vertex(p4, n1, color, 1));
			}
		}

		/**
		 * Y軸方向に高さ h、底面の半径 r1、上面の半径 r2の円錐を描画する。
		 */
		void drawCylinderY(float radius1, float radius2, float h, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices, int slices) {
			float phi = atan2(radius1 - radius2, h);

			for (int i = 0; i < slices; ++i) {
				float theta1 = M_PI * 2.0 * (float)i / slices;
				float theta2 = M_PI * 2.0 * (float)(i + 1) / slices;

				glm::vec3 p1(mat * glm::vec4(cosf(theta1) * radius1, 0, sinf(theta1) * radius1, 1));
				glm::vec3 p2(mat * glm::vec4(cosf(theta2) * radius1, 0, sinf(theta2) * radius1, 1));
				glm::vec3 p3(mat * glm::vec4(cosf(theta2) * radius2, h, sinf(theta2) * radius2, 1));
				glm::vec3 p4(mat * glm::vec4(cosf(theta1) * radius2, h, sinf(theta1) * radius2, 1));
				glm::vec3 n1(mat * glm::vec4(cosf(theta1) * cosf(phi), sinf(phi), sinf(theta1) * cosf(phi), 0));
				glm::vec3 n2(mat * glm::vec4(cosf(theta2) * cosf(phi), sinf(phi), sinf(theta2) * cosf(phi), 0));

				vertices.push_back(Vertex(p1, n1, color));
				vertices.push_back(Vertex(p2, n2, color, 1));
				vertices.push_back(Vertex(p3, n2, color));

				vertices.push_back(Vertex(p1, n1, color));
				vertices.push_back(Vertex(p3, n2, color));
				vertices.push_back(Vertex(p4, n1, color, 1));
			}
		}

		/**
		 * Z軸方向に高さ h、底面の半径 r1/r2、上面の半径 r3/r4の円錐を描画する。
		 */
		void drawCylinderZ(float radius1, float radius2, float radius3, float radius4, float h, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices, int slices, bool top_face, bool bottom_face) {
			float phi = atan2(radius1 - radius2, h);

			for (int i = 0; i < slices; ++i) {
				float theta1 = M_PI * 2.0 * (float)i / slices;
				float theta2 = M_PI * 2.0 * (float)(i + 1) / slices;

				glm::vec3 p1(mat * glm::vec4(cosf(theta1) * radius1, sinf(theta1) * radius2, 0, 1));
				glm::vec3 p2(mat * glm::vec4(cosf(theta2) * radius1, sinf(theta2) * radius2, 0, 1));
				glm::vec3 p3(mat * glm::vec4(cosf(theta2) * radius3, sinf(theta2) * radius4, h, 1));
				glm::vec3 p4(mat * glm::vec4(cosf(theta1) * radius3, sinf(theta1) * radius4, h, 1));
				glm::vec3 n1(mat * glm::vec4(cosf(theta1) * cosf(phi), sinf(theta1) * cosf(phi), sinf(phi), 0));
				glm::vec3 n2(mat * glm::vec4(cosf(theta2) * cosf(phi), sinf(theta2) * cosf(phi), sinf(phi), 0));

				vertices.push_back(Vertex(p1, n1, color));
				vertices.push_back(Vertex(p2, n2, color, 1));
				vertices.push_back(Vertex(p3, n2, color));

				vertices.push_back(Vertex(p1, n1, color));
				vertices.push_back(Vertex(p3, n2, color));
				vertices.push_back(Vertex(p4, n1, color, 1));
			}

			if (top_face) {
				drawCircle(radius3, radius4, color, glm::translate(mat, glm::vec3(0, 0, h)), vertices, slices);
			}

			if (bottom_face) {
				drawCircle(radius3, radius4, color, glm::rotate(mat, (float)M_PI, glm::vec3(1, 0, 0)), vertices, slices);
			}
		}

		void drawPrism(std::vector<glm::vec2> points, float h, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices) {
			correct(points);

			// top face
			drawConcavePolygon(points, color, glm::translate(mat, glm::vec3(0, 0, h)), vertices);

			// bottom face
			drawConcavePolygon(points, color, mat, vertices, true);

			// side faces
			for (int i = 0; i < points.size(); i++) {
				int next = (i + 1) % points.size();
				glm::vec3 p1(mat * glm::vec4(points[i], 0, 1));
				glm::vec3 p2(mat * glm::vec4(points[next], 0, 1));
				glm::vec3 p3(mat * glm::vec4(points[next], h, 1));
				glm::vec3 p4(mat * glm::vec4(points[i], h, 1));

				glm::vec3 n = glm::cross(p2 - p1, p3 - p2);
				n /= glm::length(n);

				vertices.push_back(Vertex(p1, n, color));
				vertices.push_back(Vertex(p2, n, color, 1));
				vertices.push_back(Vertex(p3, n, color));

				vertices.push_back(Vertex(p1, n, color));
				vertices.push_back(Vertex(p3, n, color));
				vertices.push_back(Vertex(p4, n, color, 1));
			}
		}

		void drawPrism(std::vector<glm::dvec2> points, double h, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices) {
			correct(points);

			// top face
			drawConcavePolygon(points, color, glm::translate(mat, glm::vec3(0, 0, h)), vertices);

			// bottom face
			drawConcavePolygon(points, color, mat, vertices, true);

			// side faces
			for (int i = 0; i < points.size(); i++) {
				int next = (i + 1) % points.size();
				glm::vec3 p1(mat * glm::vec4(points[i], 0, 1));
				glm::vec3 p2(mat * glm::vec4(points[next], 0, 1));
				glm::vec3 p3(mat * glm::vec4(points[next], h, 1));
				glm::vec3 p4(mat * glm::vec4(points[i], h, 1));

				glm::vec3 n = glm::cross(p2 - p1, p3 - p2);
				n /= glm::length(n);

				vertices.push_back(Vertex(p1, n, color));
				vertices.push_back(Vertex(p2, n, color, 1));
				vertices.push_back(Vertex(p3, n, color));

				vertices.push_back(Vertex(p1, n, color));
				vertices.push_back(Vertex(p3, n, color));
				vertices.push_back(Vertex(p4, n, color, 1));
			}
		}

		/**
		 * Create a prism mesh with the bottom polygon different from the top polygon.
		 */
		void drawPrism(std::vector<glm::dvec2> bottom_points, std::vector<glm::dvec2> top_points, double h, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices) {
			correct(bottom_points);
			correct(top_points);

			// top face
			drawConcavePolygon(top_points, color, glm::translate(mat, glm::vec3(0, 0, h)), vertices);

			// bottom face
			drawConcavePolygon(bottom_points, color, mat, vertices, true);

			// side faces
			for (int i = 0; i < bottom_points.size(); i++) {
				int next = (i + 1) % bottom_points.size();
				glm::vec3 p1(mat * glm::vec4(bottom_points[i], 0, 1));
				glm::vec3 p2(mat * glm::vec4(bottom_points[next], 0, 1));
				glm::vec3 p3(mat * glm::vec4(top_points[next], h, 1));
				glm::vec3 p4(mat * glm::vec4(top_points[i], h, 1));

				glm::vec3 n = glm::cross(p2 - p1, p3 - p2);
				n /= glm::length(n);

				vertices.push_back(Vertex(p1, n, color));
				vertices.push_back(Vertex(p2, n, color, 1));
				vertices.push_back(Vertex(p3, n, color));

				vertices.push_back(Vertex(p1, n, color));
				vertices.push_back(Vertex(p3, n, color));
				vertices.push_back(Vertex(p4, n, color, 1));
			}
		}

		/**
			* Create a prism with holes
			*/
		void drawPrismWithHoles(std::vector<glm::dvec2> points, std::vector<std::vector<glm::dvec2>> holes, double h, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices) {
			correct(points);
			for (int i = 0; i < holes.size(); i++) {
				correct(holes[i]);
			}

			// top face
			drawConcavePolygon(points, holes, color, glm::translate(mat, glm::vec3(0, 0, h)), vertices);

			// bottom face
			drawConcavePolygon(points, holes, color, mat, vertices, true);

			// side faces
			for (int i = 0; i < points.size(); i++) {
				int next = (i + 1) % points.size();
				glm::vec3 p1(mat * glm::vec4(points[i], 0, 1));
				glm::vec3 p2(mat * glm::vec4(points[next], 0, 1));
				glm::vec3 p3(mat * glm::vec4(points[next], h, 1));
				glm::vec3 p4(mat * glm::vec4(points[i], h, 1));

				glm::vec3 n = glm::cross(p2 - p1, p3 - p2);
				n /= glm::length(n);

				vertices.push_back(Vertex(p1, n, color));
				vertices.push_back(Vertex(p2, n, color, 1));
				vertices.push_back(Vertex(p3, n, color));

				vertices.push_back(Vertex(p1, n, color));
				vertices.push_back(Vertex(p3, n, color));
				vertices.push_back(Vertex(p4, n, color, 1));
			}

			// side faces for the holes
			for (int i = 0; i < holes.size(); i++) {
				reverse(holes[i].begin(), holes[i].end());
				for (int j = 0; j < holes[i].size(); j++) {
					int next = (j + 1) % holes[i].size();
					glm::vec3 p1(mat * glm::vec4(holes[i][j], 0, 1));
					glm::vec3 p2(mat * glm::vec4(holes[i][next], 0, 1));
					glm::vec3 p3(mat * glm::vec4(holes[i][next], h, 1));
					glm::vec3 p4(mat * glm::vec4(holes[i][j], h, 1));

					glm::vec3 n = glm::cross(p2 - p1, p3 - p2);
					n /= glm::length(n);

					vertices.push_back(Vertex(p1, n, color));
					vertices.push_back(Vertex(p2, n, color, 1));
					vertices.push_back(Vertex(p3, n, color));

					vertices.push_back(Vertex(p1, n, color));
					vertices.push_back(Vertex(p3, n, color));
					vertices.push_back(Vertex(p4, n, color, 1));
				}
			}
		}

		/**
		 * Z軸方向に、指定された長さ、色、半径の矢印を描画する。
		 */
		void drawArrow(float radius, float length, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices) {
			drawCylinderZ(radius, radius, radius, radius, length - radius * 4, color, mat, vertices);
			glm::mat4 m = glm::translate(mat, glm::vec3(0, 0, length - radius * 4));
			drawCylinderZ(radius * 2, radius * 2, 0, 0, radius * 4, color, m, vertices);
		}

		void drawAxes(float radius, float length, const glm::mat4& mat, std::vector<Vertex>& vertices) {
			// X軸を描画（赤色）
			glm::mat4 m1 = glm::rotate(mat, deg2rad(90), glm::vec3(0, 1, 0));
			drawArrow(radius, length, glm::vec4(1, 0, 0, 1), m1, vertices);

			// Y軸を描画（緑色）
			glm::mat4 m2 = glm::rotate(mat, deg2rad(-90), glm::vec3(1, 0, 0));
			drawArrow(radius, length, glm::vec4(0, 1, 0, 1), m2, vertices);

			// Z軸を描画 (青色）
			drawArrow(radius, length, glm::vec4(0, 0, 1, 1), mat, vertices);
		}

		void drawTube(std::vector<glm::vec3>& points, float radius, const glm::vec4& color, std::vector<Vertex>& vertices, int slices) {
			if (points.size() <= 1) return;

			glm::mat4 modelMat1, modelMat2;
			glm::vec3 x_dir, y_dir, z_dir, x_dir2, y_dir2, z_dir2;
			std::vector<glm::vec3> circle_points(slices);
			std::vector<glm::vec3> circle_normals(slices);

			{
				// 最初の円筒形の、ローカル座標系を計算
				modelMat1 = glm::translate(modelMat1, glm::vec3(points[0]));
				y_dir = glm::normalize(points[1] - points[0]);
				z_dir = glm::normalize(glm::cross(points[2] - points[1], y_dir));
				x_dir = glm::normalize(glm::cross(y_dir, z_dir));
				z_dir = glm::normalize(glm::cross(x_dir, y_dir));

				// ローカル座標系への変換行列を計算		
				modelMat1[0].x = x_dir.x; modelMat1[0].y = x_dir.y; modelMat1[0].z = x_dir.z;
				modelMat1[1].x = y_dir.x; modelMat1[1].y = y_dir.y; modelMat1[1].z = y_dir.z;
				modelMat1[2].x = z_dir.x; modelMat1[2].y = z_dir.y; modelMat1[2].z = z_dir.z;
			}

			// 円周の頂点座標を計算
			for (int k = 0; k < slices; ++k) {
				float theta = (float)k / slices * M_PI * 2.0f;

				glm::vec4 p(cosf(theta) * radius, 0, -sinf(theta) * radius, 1);
				glm::vec4 n(cosf(theta), 0, -sinf(theta), 0);
				p = modelMat1 * p;
				n = modelMat1 * n;
				circle_points[k] = glm::vec3(p);
				circle_normals[k] = glm::vec3(n);
			}

			for (int i = 0; i < points.size() - 1; ++i) {
				modelMat2 = glm::translate(glm::mat4(), glm::vec3(points[i + 1]));

				if (i < points.size() - 2) {
					// この円筒形の、ローカル座標系を計算
					y_dir2 = glm::normalize(points[i + 2] - points[i + 1]);
					if (i < points.size() - 3) {
						z_dir2 = glm::normalize(glm::cross(y_dir2, points[i + 1] - points[i]));
						if (glm::dot(z_dir, z_dir2) < 0) {
							z_dir2 = -z_dir2;
						}
					} else {
						z_dir2 = z_dir;
					}
					x_dir2 = glm::normalize(glm::cross(y_dir2, z_dir2));
					z_dir2 = glm::normalize(glm::cross(x_dir2, y_dir2));

					// ローカル座標系への変換行列を計算		
					modelMat2[0].x = x_dir2.x; modelMat2[0].y = x_dir2.y; modelMat2[0].z = x_dir2.z;
					modelMat2[1].x = y_dir2.x; modelMat2[1].y = y_dir2.y; modelMat2[1].z = y_dir2.z;
					modelMat2[2].x = z_dir2.x; modelMat2[2].y = z_dir2.y; modelMat2[2].z = z_dir2.z;
				} else {
					modelMat2 = glm::translate(glm::mat4(), points[i + 1] - points[i]) * modelMat1;
					//modelMat2 = glm::translate(modelMat1, points[i + 1] - points[i]);
				}


				float h = glm::length(points[i + 1] - points[i]);
				std::vector<glm::vec3> circle_points2(slices);
				std::vector<glm::vec3> circle_normals2(slices);

				for (int k = 0; k < slices; ++k) {
					float theta = (float)k / slices * M_PI * 2.0f;

					glm::vec4 p(cosf(theta) * radius, 0, -sinf(theta) * radius, 1);

					glm::vec3 p1 = glm::vec3(modelMat1 * p) + points[i + 1] - points[i];
					glm::vec3 p2 = glm::vec3(modelMat2 * p);
					glm::vec3 pp = (p1 + p2) * 0.5f;
				
					circle_normals2[k] = glm::normalize(pp - points[i + 1]);
					circle_points2[k] = circle_normals2[k] * radius + points[i + 1];
				}

				for (int k = 0; k < slices; ++k) {
					vertices.push_back(Vertex(glm::vec3(circle_points[k]), glm::vec3(circle_normals[k]), color));
					vertices.push_back(Vertex(glm::vec3(circle_points[(k+1)%slices]), glm::vec3(circle_normals[(k+1)%slices]), color, 1));
					vertices.push_back(Vertex(glm::vec3(circle_points2[(k+1)%slices]), glm::vec3(circle_normals2[(k+1)%slices]), color));

					vertices.push_back(Vertex(glm::vec3(circle_points[k]), glm::vec3(circle_normals[k]), color));
					vertices.push_back(Vertex(glm::vec3(circle_points2[(k+1)%slices]), glm::vec3(circle_normals2[(k+1)%slices]), color));
					vertices.push_back(Vertex(glm::vec3(circle_points2[k]), glm::vec3(circle_normals2[k]), color, 1));
				}


				modelMat1 = modelMat2;
				circle_points = circle_points2;
				circle_normals = circle_normals2;
				x_dir = x_dir2; y_dir = y_dir2; z_dir = z_dir2;
			}
		}

		void drawCurvilinearMesh(int numX, int numY, std::vector<glm::vec3>& points, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices) {
			for (int i = 0; i < numY - 1; ++i) {
				for (int j = 0; j < numX - 1; ++j) {
					glm::vec3 p1 = glm::vec3(mat * glm::vec4(points[i * numX + j], 1));
					glm::vec3 p2 = glm::vec3(mat * glm::vec4(points[i * numX + j + 1], 1));
					glm::vec3 p3 = glm::vec3(mat * glm::vec4(points[(i + 1) * numX + j + 1], 1));
					glm::vec3 p4 = glm::vec3(mat * glm::vec4(points[(i + 1) * numX + j], 1));

					glm::vec3 normal = glm::cross(p2 - p1, p3 - p1);

					vertices.push_back(Vertex(p1, normal, color));
					vertices.push_back(Vertex(p2, normal, color, 1));
					vertices.push_back(Vertex(p3, normal, color));

					vertices.push_back(Vertex(p1, normal, color));
					vertices.push_back(Vertex(p3, normal, color));
					vertices.push_back(Vertex(p4, normal, color, 1));
				}
			}
		}

		/**
		 * Make the order of points counter clock wise order
		 */
		void correct(std::vector<glm::vec2>& points, bool ccw) {
			double a = 0.0;
			for (int i = 0; i < points.size(); i++) {
				int next = (i + 1) % points.size();
				a += (points[next].x - points[i].x) * (points[next].y + points[i].y);
			}
			if (a > 0 == ccw) {
				std::reverse(points.begin(), points.end());
			}
		}

		/**
		* Make the order of points counter clock wise order
		*/
		void correct(std::vector<glm::dvec2>& points, bool ccw) {
			double a = 0.0;
			for (int i = 0; i < points.size(); i++) {
				int next = (i + 1) % points.size();
				a += (points[next].x - points[i].x) * (points[next].y + points[i].y);
			}
			if (a > 0 == ccw) {
				std::reverse(points.begin(), points.end());
			}
		}

		float deg2rad(float degree) {
			return degree * M_PI / 180.0;
		}

	}
}
