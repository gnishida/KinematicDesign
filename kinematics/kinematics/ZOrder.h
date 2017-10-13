#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <unordered_map>
#include <boost/shared_ptr.hpp>
#include <iostream>

namespace kinematics {

	class JointConnector;

	class ZOrder {
	protected:
		ZOrder() {}

	public:
		static std::vector<std::vector<int>> zorderConnectors(const std::vector<JointConnector>& connectors);
		static std::vector<int> zorderConnectors(int N, const std::vector<std::pair<int, int>>& inequalities, const std::vector<std::pair<int, int>>& differences, const std::vector<std::pair<int, std::vector<int>>>& not_betweens, int offset);
		static std::vector<int> zorderConnectors(int N, const std::vector<std::pair<int, int>>& inequalities, const std::vector<std::pair<int, int>>& differences);
		static bool hasCycle(const std::vector<std::unordered_map<int, bool>>& E);
		static bool hasCycle(const std::vector<std::unordered_map<int, bool>>& E, int u, std::vector<bool>& path, std::vector<bool>& visited);
		static bool hasPath(const std::vector<std::unordered_map<int, bool>>& E, int src, int tgt);
	};
}