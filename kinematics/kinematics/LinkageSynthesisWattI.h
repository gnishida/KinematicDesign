#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "LinkageSynthesis.h"
#include "Solution.h"
#include "BBox.h"
#include "LeastSquareSolver.h"

namespace kinematics {

	class LinkageSynthesisWattI : public LinkageSynthesis {
	public:
		LinkageSynthesisWattI() {}

	public:
		void calculateSolution(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, int num_samples, std::vector<Object25D>& fixed_body_pts, const std::vector<Object25D>& body_pts, std::vector<std::pair<double, double>>& sigmas, bool rotatable_crank, bool avoid_branch_defect, double min_link_length, std::vector<Solution>& solutions);
		bool sampleLink(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<std::vector<glm::dvec2>>& linkage_region_pts_local, const BBox& bbox_world, const std::vector<BBox>& bbox_local, std::vector<glm::dvec2>& points);
		Solution findBestSolution(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<Solution>& solutions, std::vector<Object25D>& fixed_body_pts, const std::vector<Object25D>& body_pts, double position_error_weight, double orientation_error_weight, double linkage_location_weight, double smoothness_weight, double size_weight);
		int getType(const std::vector<glm::dvec2>& points);
		bool checkRotatableCrankDefect(const std::vector<glm::dvec2>& points);
		std::pair<double, double> checkRange(const std::vector<glm::dvec2>& points);
		bool checkOrderDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points);
		bool checkBranchDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points);
		bool checkCircuitDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points);
		Kinematics constructKinematics(const std::vector<glm::dvec2>& points, const std::vector<std::vector<int>>& zorder, const std::vector<Object25D>& body_pts, bool connect_joints, std::vector<Object25D>& fixed_body_pts = std::vector<Object25D>());
		void updateBodies(Kinematics& kinematics, const std::vector<Object25D>& body_pts);

		bool checkCollision(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points, std::vector<Object25D> fixed_body_pts, const std::vector<Object25D>& body_pts, int collision_check_type);
		double check(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points);
	};

}
