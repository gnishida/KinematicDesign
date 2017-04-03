#pragma once

#include <vector>
#include "KinematicDiagram.h"
#include "Layer.h"
#include "SimpleGraph.h"

namespace kinematics {

	class TreeNode {
	public:
		std::vector<boost::shared_ptr<Link>> links;
		std::vector<std::pair<std::vector<boost::shared_ptr<Joint>>, boost::shared_ptr<TreeNode>>> child_nodes;
		boost::shared_ptr<TreeNode> parent;
		std::vector<boost::shared_ptr<Joint>> parent_joint;
		std::vector<glm::dmat4x4> mat;

	public:
		TreeNode() {}
		bool hasGrandParent();
	};

	class LinkageSolver {
	public:
		LinkageSolver();

		static void adjustSketch(std::vector<canvas::Layer>& layers);
		static void translatePart(std::vector<canvas::Layer>& layers, int shape_id, std::vector<glm::dvec2> offset, QMap<int, bool> visited, const SimpleGraph& mst);
		static std::vector<KinematicDiagram> initialKinematicDiagram(std::vector<canvas::Layer> layers);
		static SimpleGraph constructGraph(std::vector<canvas::Layer> layers);
		static double findShortestDistanceBetweenLinks(boost::shared_ptr<Link> link0a, boost::shared_ptr<Link> link0b, boost::shared_ptr<Link> link1a, boost::shared_ptr<Link> link1b, double& best_k, double& best_l);
		static KinematicDiagram solve(std::vector<KinematicDiagram> initial_diagrams, std::vector<double>& params);
		static boost::shared_ptr<TreeNode> constructTree(std::vector<KinematicDiagram> diagrams);
		static void constructSubTree(boost::shared_ptr<TreeNode> parent);
		static boost::shared_ptr<TreeNode> findGroundLink(std::vector<KinematicDiagram> diagrams);
		static KinematicDiagram optimize(std::vector<KinematicDiagram> initial_diagrams);
	};

}