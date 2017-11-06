#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "LinkageSynthesis.h"
#include "Solution.h"
#include "BBox.h"

namespace kinematics {

	class LinkageSynthesis4R : public LinkageSynthesis {
	public:
		LinkageSynthesis4R() {}

	public:
		void calculateSolution(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, int num_samples, std::vector<Object25D>& fixed_body_pts, const Object25D& body_pts, std::vector<std::pair<double, double>>& sigmas, bool rotatable_crank, bool avoid_branch_defect, double min_link_length, std::vector<Solution>& solutions);
		bool sampleLink(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_region_pts_local, const BBox& bbox_world, const BBox& bbox_local, glm::dvec2& A0, glm::dvec2& A1);
		bool sampleLinkForThreePoses(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_region_pts_local, const BBox& bbox, glm::dvec2& A0, glm::dvec2& A1);
		bool sampleLinkForTwoPoses(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_region_pts_local, const BBox& bbox_world, const BBox& bbox_local, glm::dvec2& A0, glm::dvec2& A1);
		Solution findBestSolution(const std::vector<glm::dmat3x3>& poses, const std::vector<Solution>& solutions, std::vector<Object25D>& fixed_body_pts, const Object25D& body_pts, double position_error_weight, double orientation_error_weight, double linkage_location_weight, double smoothness_error, double size_weight);
		Kinematics constructKinematics(const std::vector<glm::dvec2>& points, const std::vector<std::vector<int>>& zorder, const Object25D& body_pts, bool connect_joints, std::vector<Object25D>& fixed_body_pts = std::vector<Object25D>());

		int getType(const std::vector<glm::dvec2>& points);
		bool checkFolding(const std::vector<glm::dvec2>& points);
		std::pair<double, double> checkRange(const std::vector<glm::dvec2>& points);
		bool checkRotatableCrankDefect(const std::vector<glm::dvec2>& points);
		bool checkOrderDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points);
		bool checkBranchDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points);
		bool checkCircuitDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points);
		bool checkCollision(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points, std::vector<Object25D> fixed_body_pts, const Object25D& body_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, int collision_check_type);
		Kinematics recordCollisionForConnectors(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points, std::vector<Object25D> fixed_body_pts, const Object25D& body_pts);
		double tortuosityOfTrajectory(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points, const Object25D& body_pts);
	};

}