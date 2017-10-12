#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace kinematics {

	class Solution {
	public:
		std::vector<glm::dvec2> points;
		double position_error;
		double orientation_error;
		double dist;	// how far the linkage is from the user-specified region
		std::vector<glm::dmat3x3> poses;
		std::vector<std::vector<int>> zorder;

	public:
		Solution() {}
		Solution(const std::vector<glm::dvec2>& points, double position_error, double orientation_error, double dist);
		Solution(const std::vector<glm::dvec2>& points, double pose_error, double orientation_error, double dist, const std::vector<glm::dmat3x3>& poses);
		Solution(const std::vector<glm::dvec2>& points, double pose_error, double orientation_error, double dist, const std::vector<glm::dmat3x3>& poses, const std::vector<std::vector<int>>& zorder);
	};

}