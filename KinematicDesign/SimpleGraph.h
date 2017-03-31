#pragma once

#include <vector>
#include <QMap>
#include <boost/shared_ptr.hpp>

namespace kinematics {

	class SimpleGraphVertex {
	public:
		int id;
		std::vector<int> edges;

	public:
		SimpleGraphVertex(int id);
	};

	class SimpleGraphEdge {
	public:
		int id;
		std::vector<int> vertices;
		double weight;

	public:
		SimpleGraphEdge(int id, double weight, int vertex_id1, int vertex_id2);
	};

	class SimpleGraph {
	public:
		QMap<int, boost::shared_ptr<SimpleGraphVertex>> vertices;
		QMap<int, boost::shared_ptr<SimpleGraphEdge>> edges;

	public:
		SimpleGraph();
		~SimpleGraph();

		void clear();
		boost::shared_ptr<SimpleGraphVertex> addVertex(int id);
		boost::shared_ptr<SimpleGraphEdge> addEdge(double weight, int vertex1_id, int vertex2_id);
		boost::shared_ptr<SimpleGraphEdge> addEdge(int id, double weight, boost::shared_ptr<SimpleGraphVertex> vertex1, boost::shared_ptr<SimpleGraphVertex> vertex2);

		bool isConnected(int vertex_id1, int vertex_id2);
		bool isNeighbor(int vertex_id1, int vertex_id2);
		bool canReach(std::vector<int> history, int end);
		SimpleGraph minimumSpanningTree();
		
	};

}