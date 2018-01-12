#include "Solution.h"

namespace kinematics {

	Solution::Solution(int linkage_type, const std::vector<glm::dvec2>& points, double position_error, double orientation_error, const std::vector<glm::dmat3x3>& poses) {
		this->linkage_type = linkage_type;
		this->points = points;
		this->position_error = position_error;
		this->orientation_error = orientation_error;
		this->poses = poses;
		this->cost = std::numeric_limits<double>::max();
	}

	Solution::Solution(int linkage_type, const std::vector<glm::dvec2>& points, double position_error, double orientation_error, const std::vector<glm::dmat3x3>& poses, const std::vector<std::vector<int>>& zorder) {
		this->linkage_type = linkage_type;
		this->points = points;
		this->position_error = position_error;
		this->orientation_error = orientation_error;
		this->poses = poses;
		this->zorder = zorder;
		this->cost = std::numeric_limits<double>::max();
	}
}