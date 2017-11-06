#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "Solution.h"
#include "BBox.h"
#include "KinematicUtils.h"
#include "Kinematics.h"

namespace kinematics {

	class LinkageSynthesis {
	protected:
		LinkageSynthesis() {}

	public:
		virtual void calculateSolution(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, int num_samples, std::vector<Object25D>& fixed_body_pts, const Object25D& body_pts, std::vector<std::pair<double, double>>& sigmass, bool rotatable_crank, bool avoid_branch_defect, double min_link_length, std::vector<Solution>& solutions) = 0;
		virtual Solution findBestSolution(const std::vector<glm::dmat3x3>& poses, const std::vector<Solution>& solutions, std::vector<Object25D>& fixed_body_pts, const Object25D& body_pts, double position_error_weight, double orientation_error_weight, double linkage_location_weight, double smoothness_weight, double size_weight) = 0;
		virtual int getType(const std::vector<glm::dvec2>& points) = 0;
		virtual bool checkOrderDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points) = 0;
		virtual bool checkBranchDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points) = 0;
		virtual bool checkCircuitDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points) = 0;
		virtual Kinematics constructKinematics(const std::vector<glm::dvec2>& points, const std::vector<std::vector<int>>& zorder, const Object25D& body_pts, bool connect_joints, std::vector<Object25D>& fixed_body_pts = std::vector<Object25D>()) = 0;
	};

}