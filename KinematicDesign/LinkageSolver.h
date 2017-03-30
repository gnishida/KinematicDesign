#pragma once

#include <vector>
#include "KinematicDiagram.h"
#include "Layer.h"

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

		static std::vector<KinematicDiagram> initialKinematicDiagram(std::vector<canvas::Layer> layers);
		static double findShortestDistanceBetweenLinks(boost::shared_ptr<Link> link0a, boost::shared_ptr<Link> link0b, boost::shared_ptr<Link> link1a, boost::shared_ptr<Link> link1b, double& best_k, double& best_l);
		static KinematicDiagram solve(std::vector<KinematicDiagram> initial_diagrams);
		static boost::shared_ptr<TreeNode> constructTree(std::vector<KinematicDiagram> diagrams);
		static void constructSubTree(boost::shared_ptr<TreeNode> parent);
		static boost::shared_ptr<TreeNode> findGroundLink(std::vector<KinematicDiagram> diagrams);
	};

}