#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "BBox.h"

namespace kinematics {

	class Solution {
	public:
		std::vector<glm::dvec2> points;
		double position_error;
		double orientation_error;
		double dist;	// how far the linkage is from the user-specified region
		std::vector<glm::dmat3x3> poses;
		std::vector<glm::dvec2> linkage_region;
		BBox linkage_region_bbox;
		std::vector<std::vector<int>> zorder;

	public:
		Solution() {}
		Solution(const std::vector<glm::dvec2>& points, double position_error, double orientation_error, double dist);
		Solution(const std::vector<glm::dvec2>& points, double position_error, double orientation_error, double dist, const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region, const BBox& linkage_region_bbox);
		Solution(const std::vector<glm::dvec2>& points, double position_error, double orientation_error, double dist, const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region, const BBox& linkage_region_bbox, const std::vector<std::vector<int>>& zorder);
	};

}