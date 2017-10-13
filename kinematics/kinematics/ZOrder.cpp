#include "ZOrder.h"
#include "JointConnector.h"
#include <queue>

namespace kinematics {

	std::vector<std::vector<int>> ZOrder::zorderConnectors(const std::vector<JointConnector>& connectors) {
		std::vector<std::pair<int, int>> inequalities;
		std::vector<std::pair<int, std::vector<int>>> not_betweens;
		std::vector<std::pair<int, int>> differences;

		for (int i = 0; i < connectors.size(); i++) {
			for (auto it = connectors[i].collisions1.begin(); it != connectors[i].collisions1.end(); it++) {
				int j = it.key();
				inequalities.push_back(std::make_pair(i, j));
			}

			for (auto it = connectors[i].collisions2.begin(); it != connectors[i].collisions2.end(); it++) {
				not_betweens.push_back(std::make_pair(i, it.value()));
			}

			for (auto it = connectors[i].neighbors.begin(); it != connectors[i].neighbors.end(); it++) {
				int j = it.key();
				differences.push_back(std::make_pair(i, j));
			}
		}

		std::vector<int> results = zorderConnectors(connectors.size(), inequalities, differences, not_betweens, 0);

		std::vector<std::vector<int>> ans(3);
		int k = 0;
		for (int i = 0; i < connectors.size(); i++) {
			ans[connectors[i].type].push_back(results[k++]);
		}
		return ans;
	}

	std::vector<int> ZOrder::zorderConnectors(int N, const std::vector<std::pair<int, int>>& inequalities, const std::vector<std::pair<int, int>>& differences, const std::vector<std::pair<int, std::vector<int>>>& not_betweens, int offset) {
		if (offset == not_betweens.size()) {
			return zorderConnectors(N, inequalities, differences);
		}
		else {
			int u = not_betweens[offset].first;

			std::vector<std::pair<int, int>> inequalities2 = inequalities;
			for (int i = 0; i < not_betweens[offset].second.size(); i++) {
				int v = not_betweens[offset].second[i];
				inequalities2.push_back(std::make_pair(u, v));
			}

			try {
				return zorderConnectors(N, inequalities2, differences, not_betweens, offset + 1);
			}
			catch (char* ex) {
			}

			std::vector<std::pair<int, int>> inequalities3 = inequalities;
			for (int i = 0; i < not_betweens[offset].second.size(); i++) {
				int v = not_betweens[offset].second[i];
				inequalities3.push_back(std::make_pair(v, u));
			}

			return zorderConnectors(N, inequalities3, differences, not_betweens, offset + 1);
		}
	}

	/**
	 * Use the topological sorting to determine the z-order of connectors.
	 *
	 * @param inequalities		(i, j) where z(i) > z(j)
	 * @param differences		(i, j) where z(i) != z(j)
	 */
	std::vector<int> ZOrder::zorderConnectors(int N, const std::vector<std::pair<int, int>>& inequalities, const std::vector<std::pair<int, int>>& differences) {
		// build a graph based on inequalities
		std::vector<std::unordered_map<int, bool>> outE(N);
		std::vector<std::unordered_map<int, bool>> inE(N);
		for (int i = 0; i < inequalities.size(); i++) {
			int u = inequalities[i].first;
			int v = inequalities[i].second;

			outE[u][v] = true;
			inE[v][u] = true;
		}

		// check a cycle
		if (hasCycle(outE)) {
			throw "No appropriate z-order was found.";
		}

		// add non-equalities to the graph
		for (int i = 0; i < differences.size(); i++) {
			int u = differences[i].first;
			int v = differences[i].second;

			if (hasPath(outE, u, v) || hasPath(outE, v, u)) continue;

			outE[v][u] = true;
			inE[u][v] = true;
		}

		// now, the problem becomes a standard topological sorting.
		std::vector<int> ans(N, -1);
		std::queue<int> Q;
		int z = 1;
		for (int i = 0; i < N; i++) {
			if (outE[i].size() == 0) {
				ans[i] = z;
				Q.push(i);
			}
		}
		while (!Q.empty()) {
			int M = Q.size();
			z++;
			for (int i = 0; i < M; i++) {
				int u = Q.front();
				Q.pop();

				for (auto it = inE[u].begin(); it != inE[u].end(); it++) {
					int v = it->first;
					ans[v] = z;
					Q.push(v);
				}
			}
		}

		return ans;
	}

	bool ZOrder::hasCycle(const std::vector<std::unordered_map<int, bool>>& E) {
		int N = E.size();
		std::vector<bool> visited(N, false);
		std::vector<bool> path(N, false);

		for (int i = 0; i < N; i++) {
			if (visited[i]) continue;

			visited[i] = true;
			path[i] = true;

			if (hasCycle(E, i, path, visited)) return true;
			visited[i] = false;
			path[i] = false;
		}

		return false;
	}

	/**
	 * Check if there is a cycle traversing from the node u.
	 *
	 * @param path		the nodes that are on the current traversal
	 * @param visited	the nodes that are already checked for the cycle
	 * @return			true if there is a cycle
	 */
	bool ZOrder::hasCycle(const std::vector<std::unordered_map<int, bool>>& E, int u, std::vector<bool>& path, std::vector<bool>& visited) {
		for (auto it = E[u].begin(); it != E[u].end(); it++) {
			int v = it->first;
			if (path[v]) return true;
			if (visited[v]) continue;
			visited[v] = true;
			path[v] = true;
			if (hasCycle(E, v, path, visited)) return true;
			//visited[v] = false;
			path[v] = false;
		}
		return false;
	}

	bool ZOrder::hasPath(const std::vector<std::unordered_map<int, bool>>& E, int src, int tgt) {
		int N = E.size();
		std::vector<bool> visited(N, false);
		visited[src] = true;
		std::queue<int> Q;
		Q.push(src);
		while (!Q.empty()) {
			int u = Q.front();
			Q.pop();

			for (auto it = E[u].begin(); it != E[u].end(); it++) {
				int v = it->first;
				if (visited[v]) continue;
				if (v == tgt) return true;

				visited[v] = true;
				Q.push(v);
			}
		}

		return false;
	}

}