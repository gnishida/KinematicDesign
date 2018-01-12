#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "BBox.h"

namespace kinematics {

	class Solution {
	public:
		int linkage_type;
		std::vector<glm::dvec2> points;
		double position_error;
		double orientation_error;
		std::vector<glm::dmat3x3> poses;
		std::vector<std::vector<int>> zorder;
		double cost;

	public:
		Solution() : cost(std::numeric_limits<double>::max()) {}
		Solution(int linkage_type, const std::vector<glm::dvec2>& points, double position_error, double orientation_error, const std::vector<glm::dmat3x3>& poses);
		Solution(int linkage_type, const std::vector<glm::dvec2>& points, double position_error, double orientation_error, const std::vector<glm::dmat3x3>& poses, const std::vector<std::vector<int>>& zorder);
	};

}