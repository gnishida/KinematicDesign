#include "SimpleGraph.h"
#include <iostream>

namespace kinematics {

	bool sortByValue(const std::pair<int, double>& a, const std::pair<int, double>& b) {
		return a.second < b.second;
	}

	SimpleGraphVertex::SimpleGraphVertex(int id) {
		this->id = id;
	}

	SimpleGraphEdge::SimpleGraphEdge(int id, double weight, int vertex_id1, int vertex_id2) {
		this->id = id;
		this->weight = weight;
		vertices.push_back(vertex_id1);
		vertices.push_back(vertex_id2);
	}

	SimpleGraph::SimpleGraph() {
	}


	SimpleGraph::~SimpleGraph() {
	}

	void SimpleGraph::clear() {
		vertices.clear();
		edges.clear();
	}

	boost::shared_ptr<SimpleGraphVertex> SimpleGraph::addVertex(int id) {
		if (id == -1) {
			if (vertices.empty()) {
				id = 0;
			}
			else {
				id = vertices.lastKey() + 1;
			}
		}

		boost::shared_ptr<SimpleGraphVertex> vertex = boost::shared_ptr<SimpleGraphVertex>(new SimpleGraphVertex(id));
		vertices[id] = vertex;

		return vertex;
	}

	boost::shared_ptr<SimpleGraphEdge> SimpleGraph::addEdge(double weight, int vertex1_id, int vertex2_id) {
		return addEdge(-1, weight, vertices[vertex1_id], vertices[vertex2_id]);
	}

	boost::shared_ptr<SimpleGraphEdge> SimpleGraph::addEdge(int id, double weight, boost::shared_ptr<SimpleGraphVertex> vertex1, boost::shared_ptr<SimpleGraphVertex> vertex2) {
		if (id == -1) {
			if (edges.empty()) {
				id = 0;
			}
			else {
				id = edges.lastKey() + 1;
			}
		}

		boost::shared_ptr<SimpleGraphEdge> edge = boost::shared_ptr<SimpleGraphEdge>(new SimpleGraphEdge(id, weight, vertex1->id, vertex2->id));
		edges[id] = edge;
		vertex1->edges.push_back(id);
		vertex2->edges.push_back(id);

		return edge;
	}

	bool SimpleGraph::isConnected(int vertex_id1, int vertex_id2) const {
		std::vector<int> history1;
		history1.push_back(vertex_id1);
		if (canReach(history1, vertex_id2)) return true;

		std::vector<int> history2;
		history2.push_back(vertex_id2);
		if (canReach(history2, vertex_id1)) return true;
		
		return false;
	}

	bool SimpleGraph::isNeighbor(int vertex_id1, int vertex_id2) const {
		for (int i = 0; i < vertices[vertex_id1]->edges.size(); ++i) {
			int edge_id = vertices[vertex_id1]->edges[i];
			for (int j = 0; j < edges[edge_id]->vertices.size(); ++j) {
				int v = edges[edge_id]->vertices[j];
				if (v == vertex_id2) return true;
			}
		}

		return false;
	}

	boost::shared_ptr<SimpleGraphEdge> SimpleGraph::getEdge(int vertex_id1, int vertex_id2) const {
		for (int i = 0; i < vertices[vertex_id1]->edges.size(); ++i) {
			int edge_id = vertices[vertex_id1]->edges[i];

			for (int j = 0; j < edges[edge_id]->vertices.size(); ++j) {
				if (edges[edge_id]->vertices[j] == vertex_id2) {
					return edges[edge_id];
				}
			}
		}

		throw "No edge.";
	}

	bool SimpleGraph::canReach(std::vector<int> history, int end) const {
		for (int i = 0; i < vertices[history.back()]->edges.size(); ++i) {
			int edge_id = vertices[history.back()]->edges[i];

			for (int j = 0; j < edges[edge_id]->vertices.size(); ++j) {
				int v = edges[edge_id]->vertices[j];
				if (v == end) return true;

				bool loop_detected = false;
				for (int k = 0; k < history.size(); ++k) {
					if (v == history[k]) loop_detected = true;
				}
				if (loop_detected) continue;

				std::vector<int> next_history = history;
				next_history.push_back(v);
				if (canReach(next_history, end)) return true;
			}
		}

		return false;
	}

	SimpleGraph SimpleGraph::minimumSpanningTree() const {
		SimpleGraph mst;

		// copy all the vertices
		for (auto it = vertices.begin(); it != vertices.end(); ++it) {
			mst.addVertex(it.key());
		}

		// sort the edges by their weights
		std::vector<std::pair<int, double>> sortedEdges;
		for (int i = 0; i < edges.size(); ++i) {
			sortedEdges.push_back(std::make_pair(edges[i]->id, edges[i]->weight));
		}
		std::sort(sortedEdges.begin(), sortedEdges.end(), sortByValue);

		// 
		for (int i = 0; i < sortedEdges.size(); ++i) {
			int edge_id = sortedEdges[i].first;
			bool isolated = false;

			assert(edges[edge_id]->vertices.size() == 2);

			int vertex_id1 = edges[edge_id]->vertices[0];
			int vertex_id2 = edges[edge_id]->vertices[1];

			if (!mst.isConnected(vertex_id1, vertex_id2)) {
				mst.addEdge(edges[edge_id]->weight, vertex_id1, vertex_id2);
			}
		}

		return mst;
	}

}