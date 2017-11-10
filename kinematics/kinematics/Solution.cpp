#include "Solution.h"

namespace kinematics {

	Solution::Solution(const std::vector<glm::dvec2>& points, double position_error, double orientation_error, const std::vector<glm::dmat3x3>& poses) {
		this->points = points;
		this->position_error = position_error;
		this->orientation_error = orientation_error;
		this->poses = poses;
	}

	Solution::Solution(const std::vector<glm::dvec2>& points, double position_error, double orientation_error, const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region, const BBox& linkage_region_bbox) {
		this->points = points;
		this->position_error = position_error;
		this->orientation_error = orientation_error;
		this->poses = poses;
		this->linkage_region = linkage_region;
		this->linkage_region_bbox = linkage_region_bbox;
	}

	Solution::Solution(const std::vector<glm::dvec2>& points, double position_error, double orientation_error, const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region, const BBox& linkage_region_bbox, const std::vector<std::vector<int>>& zorder) {
		this->points = points;
		this->position_error = position_error;
		this->orientation_error = orientation_error;
		this->poses = poses;
		this->linkage_region = linkage_region;
		this->linkage_region_bbox = linkage_region_bbox;
		this->zorder = zorder;
	}
}