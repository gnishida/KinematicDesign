#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include "Vertex.h"
#include <vector>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/partition_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/random_polygon_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/random_polygon_2.h>
#include <CGAL/Polygon_2.h>

namespace kinematics {
	namespace glutils {

		const double M_PI = 3.1415926535897932384626433832795028841971693993751;

		class BoundingBox {
		public:
			glm::dvec3 minPt;
			glm::dvec3 maxPt;

		public:
			BoundingBox();
			BoundingBox(const std::vector<glm::dvec2>& points);
			BoundingBox(const std::vector<glm::dvec3>& points);
			BoundingBox(const std::vector<std::vector<glm::dvec3>>& points);
			void addPoint(const glm::dvec2& point);
			void addPoint(const glm::dvec3& point);
			double sx() const { return maxPt.x - minPt.x; }
			double sy() const { return maxPt.y - minPt.y; }
			double sz() const { return maxPt.z - minPt.z; }
			glm::dvec3 center() const { return (maxPt + minPt) * 0.5; }
			bool contains(const glm::dvec2& point, double threshold) const;
			bool contains(const glm::dvec3& point, double threshold) const;
		};

		// The following definitions are for triangulation only.
		struct FaceInfo2 {
			FaceInfo2() {}
			int nesting_level;
			bool in_domain(){
				return nesting_level % 2 == 1;
			}
		};

		typedef CGAL::Exact_predicates_inexact_constructions_kernel       K;
		typedef CGAL::Triangulation_vertex_base_2<K>                      Vb;
		typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2, K>    Fbb;
		typedef CGAL::Constrained_triangulation_face_base_2<K, Fbb>        Fb;
		typedef CGAL::Triangulation_data_structure_2<Vb, Fb>               TDS;
		typedef CGAL::Exact_predicates_tag                                Itag;
		typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>  CDT;
		typedef CDT::Point                                                Point;
		typedef CGAL::Partition_traits_2<K>                         Traits;
		typedef Traits::Polygon_2                                   Polygon_2;
		typedef Traits::Point_2                                     Point_2;
		typedef Polygon_2::Vertex_iterator                          Vertex_iterator;
		typedef std::list<Polygon_2>                                Polygon_list;
		typedef CGAL::Creator_uniform_2<int, Point_2>               Creator;
		typedef CGAL::Random_points_in_square_2< Point_2, Creator > Point_generator;
		typedef boost::shared_ptr<Polygon_2>						PolygonPtr;

		// geometry computation
		float crossProduct(const glm::vec2& a, const glm::vec2& b);
		bool isWithinPolygon(const glm::vec2& p, const std::vector<glm::vec2>& points);
		bool isWithinPolygon(const glm::dvec2& p, const std::vector<glm::dvec2>& points);
		float area(const std::vector<glm::vec2>& points);
		void offsetPolygon(const std::vector<glm::vec2>& points, float offsetDistance, std::vector<glm::vec2>& offset_points);
		float distance(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c, bool segmentOnly = false);
		float distance(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);
		glm::vec3 lineLineIntersection(const glm::vec3& p1, const glm::vec3& v1, const glm::vec3& p2, const glm::vec3& v2, float weight1 = 0.5f, float weight2 = 0.5f);
		glm::vec3 rayPlaneIntersection(const glm::vec3& a, const glm::vec3& v, const glm::vec3& p, const glm::vec3& n);
		bool rayTriangleIntersection(const glm::vec3& a, const glm::vec3& v, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, glm::vec3& intPt);
		glm::vec2 barycentricCoordinates(const glm::vec2& p1, const glm::vec2& p2, const glm::vec2& p3, const glm::vec2& p);

		// mesh generation
		void drawCircle(float r1, float r2, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices, int slices = 12);
		void drawCircle(float r1, float r2, float texWidth, float texHeight, const glm::mat4& mat, std::vector<Vertex>& vertices, int slices = 12);
		void drawQuad(float w, float h, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices);
		void drawQuad(float w, float h, const glm::vec2& t1, const glm::vec2& t2, const glm::vec2& t3, const glm::vec2& t4, const glm::mat4& mat, std::vector<Vertex>& vertices);
		void drawPolygon(const std::vector<glm::vec3>& points, const glm::vec4& color, const std::vector<glm::vec2>& texCoords, const glm::mat4& mat, std::vector<Vertex>& vertices);
		void drawPolygon(const std::vector<glm::vec3>& points, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices);
		void drawPolygon(const std::vector<glm::vec2>& points, const glm::vec4& color, const std::vector<glm::vec2>& texCoords, const glm::mat4& mat, std::vector<Vertex>& vertices, bool flip = false);
		void drawPolygon(const std::vector<glm::vec2>& points, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices, bool flip = false);
		void drawConcavePolygon(const std::vector<glm::vec2>& points, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices, bool flip = false);
		void drawConcavePolygon(const std::vector<glm::dvec2>& points, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices, bool flip = false);
		void mark_domains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border);
		void mark_domains(CDT& cdt);
		void drawConcavePolygon(const std::vector<glm::dvec2>& points, const std::vector<std::vector<glm::dvec2>>& holes, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices, bool flip = false);
		void drawConcavePolygon(const std::vector<glm::vec2>& points, const glm::vec4& color, const std::vector<glm::vec2>& texCoords, const glm::mat4& mat, std::vector<Vertex>& vertices, bool flip = false);
		void drawGrid(float width, float height, float cell_size, const glm::vec4& lineColor, const glm::vec4& backgroundColor, const glm::mat4& mat, std::vector<Vertex>& vertices);
		void drawBox(float length_x, float length_y, float length_z, glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices);
		void drawSphere(float radius, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices);
		void drawEllipsoid(float r1, float r2, float r3, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices);
		void drawCylinderX(float radius1, float radius2, float h, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices, int slices = 12);
		void drawCylinderY(float radius1, float radius2, float h, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices, int slices = 12);
		void drawCylinderZ(float radius1, float radius2, float radius3, float radius4, float h, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices, int slices = 12, bool top_face = true, bool bottom_face = true);
		void drawPrism(std::vector<glm::vec2> points, float h, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices);
		void drawPrism(std::vector<glm::dvec2> points, double h, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices);
		void drawPrism(std::vector<glm::dvec2> bottom_points, std::vector<glm::dvec2> top_points, double h, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices);
		void drawPrismWithHoles(std::vector<glm::dvec2> points, std::vector<std::vector<glm::dvec2>> holes, double h, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices);
		void drawArrow(float radius, float length, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices);
		void drawAxes(float radius, float length, const glm::mat4& mat, std::vector<Vertex>& vertices);
		void drawTube(std::vector<glm::vec3>& points, float radius, const glm::vec4& color, std::vector<Vertex>& vertices, int slices = 12);
		void drawCurvilinearMesh(int numX, int numY, std::vector<glm::vec3>& points, const glm::vec4& color, const glm::mat4& mat, std::vector<Vertex>& vertices);

		void correct(std::vector<glm::vec2>& points, bool ccw = true);
		void correct(std::vector<glm::dvec2>& points, bool ccw = true);

		float deg2rad(float degree);

	}
}