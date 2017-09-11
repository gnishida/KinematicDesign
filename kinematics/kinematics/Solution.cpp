#include "Solution.h"

namespace kinematics {

	Solution::Solution(const glm::dvec2& fixed_point1, const glm::dvec2& moving_point1, const glm::dvec2& fixed_point2, const glm::dvec2& moving_point2, double position_error, double orientation_error, double dist) {
		this->fixed_point[0] = fixed_point1;
		this->moving_point[0] = moving_point1;
		this->fixed_point[1] = fixed_point2;
		this->moving_point[1] = moving_point2;
		this->position_error = position_error;
		this->orientation_error = orientation_error;
		this->dist = dist;
	}

	Solution::Solution(const glm::dvec2& fixed_point1, const glm::dvec2& moving_point1, const glm::dvec2& fixed_point2, const glm::dvec2& moving_point2, double position_error, double orientation_error, double dist, const std::vector<glm::dmat3x3>& poses) {
		this->fixed_point[0] = fixed_point1;
		this->moving_point[0] = moving_point1;
		this->fixed_point[1] = fixed_point2;
		this->moving_point[1] = moving_point2;
		this->position_error = position_error;
		this->orientation_error = orientation_error;
		this->dist = dist;
		this->poses = poses;
	}

}