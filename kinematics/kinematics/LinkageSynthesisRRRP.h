#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "LinkageSynthesis.h"
#include "Solution.h"
#include "BBox.h"

namespace kinematics {

	class LinkageSynthesisRRRP : public LinkageSynthesis {
	public:
		LinkageSynthesisRRRP(const std::vector<Object25D>& fixed_bodies, const std::vector<std::pair<double, double>>& sigmas, bool avoid_branch_defect, double min_transmission_angle, double min_link_length, const std::vector<double>& weights);

	public:
		void calculateSolution(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, int num_samples, const Object25D& moving_body, std::vector<Solution>& solutions);
		bool optimizeCandidate(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const BBox& bbox, std::vector<glm::dvec2>& points);
		bool optimizeLink(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const BBox& bbox, glm::dvec2& A0, glm::dvec2& A1);
		bool optimizeLinkForThreePoses(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, glm::dvec2& A0, glm::dvec2& A1);
		bool optimizeLinkForTwoPoses(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, glm::dvec2& A0, glm::dvec2& A1);
		bool optimizeSlider(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const BBox& bbox, glm::dvec2& A0, glm::dvec2& A1);
		Solution findBestSolution(const std::vector<glm::dmat3x3>& poses, std::vector<Solution>& solutions, const std::vector<glm::dvec2>& linkage_region_pts, const cv::Mat& dist_map, const BBox& dist_map_bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, const Object25D& moving_body, int num_particles, int num_iterations, bool record_file);
		double calculateCost(Solution& solution, const Object25D& moving_body, const cv::Mat& dist_map, const BBox& dist_map_bbox);
		Kinematics constructKinematics(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points, const std::vector<std::vector<int>>& zorder, const Object25D& moving_body, bool connect_joints, const std::vector<Object25D>& fixed_bodies, std::vector<glm::dvec2>& connected_pts);
		void updateMovingBodies(Kinematics& kinematics, const Object25D& moving_body);

		bool checkHardConstraints(std::vector<glm::dvec2>& points, const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, const Object25D& moving_body, std::vector<std::vector<int>>& zorder);
		int getType(const std::vector<glm::dvec2>& points);
		std::pair<double, double> checkRange(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points);
		bool checkOrderDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points);
		bool checkBranchDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points);
		bool checkCircuitDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points);
		bool checkCollision(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points, const std::vector<Object25D>& fixed_bodies, const Object25D& moving_body, const std::vector<glm::dvec2>& linkage_avoidance_pts, glm::dvec2& slider_end_pos1, glm::dvec2& slider_end_pos2);
		Kinematics recordCollisionForConnectors(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points, const std::vector<Object25D> fixed_bodies, const Object25D& moving_body);
		double tortuosityOfTrajectory(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points, const Object25D& moving_body);

		void generate3DGeometry(const Kinematics& kinematics, std::vector<Vertex>& vertices);
		void saveSCAD(const QString& dirname, int index, const Kinematics& kinematics);
		void saveSTL(const QString& dirname, const std::vector<Kinematics>& kinematics);
	};

}