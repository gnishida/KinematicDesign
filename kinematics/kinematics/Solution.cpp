#include "Solution.h"

namespace kinematics {

	Solution::Solution(const std::vector<glm::dvec2>& points, double position_error, double orientation_error, double dist) {
		this->points = points;
		this->position_error = position_error;
		this->orientation_error = orientation_error;
		this->dist = dist;
	}

	Solution::Solution(const std::vector<glm::dvec2>& points, double position_error, double orientation_error, double dist, const std::vector<glm::dmat3x3>& poses) {
		this->points = points;
		this->position_error = position_error;
		this->orientation_error = orientation_error;
		this->dist = dist;
		this->poses = poses;
	}

}