#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace kinematics {

	class Solution {
	public:
		glm::dvec2 fixed_point[2];
		glm::dvec2 moving_point[2];
		double pose_error;
		std::vector<glm::dmat3x3> poses;

	public:
		Solution() {}
		Solution(const glm::dvec2& fixed_point1, const glm::dvec2& moving_point1, const glm::dvec2& fixed_point2, const glm::dvec2& moving_point2, double pose_error);
		Solution(const glm::dvec2& fixed_point1, const glm::dvec2& moving_point1, const glm::dvec2& fixed_point2, const glm::dvec2& moving_point2, double pose_error, const std::vector<glm::dmat3x3>& poses);
	};

}