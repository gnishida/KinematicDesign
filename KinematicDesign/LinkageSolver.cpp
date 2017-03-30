#include "LinkageSolver.h"
#include "PinJoint.h"
#include "Link.h"
#include "BoundingBox.h"
#include "Utils.h"

namespace kinematics {

	bool TreeNode::hasGrandParent() {
		if (!parent) return false;
		else if (!parent->parent) return false;
		else return true;
	}

	LinkageSolver::LinkageSolver() {
	}

	std::vector<KinematicDiagram> LinkageSolver::initialKinematicDiagram(std::vector<canvas::Layer> layers) {
		std::vector<KinematicDiagram> tmp_diagrams;

		for (int i = 0; i < layers.size(); ++i) {
			KinematicDiagram diagram;

			for (int j = 0; j < layers[i].shapes.size(); ++j) {
				canvas::BoundingBox bbox = layers[i].shapes[j]->boundingBox();
				glm::dvec2 p1, p2;
				if (bbox.width() >= bbox.height()) {
					p1 = glm::dvec2(bbox.minPt.x, (bbox.minPt.y + bbox.maxPt.y) * 0.5);
					p2 = glm::dvec2(bbox.maxPt.x, (bbox.minPt.y + bbox.maxPt.y) * 0.5);
				}
				else {
					p1 = glm::dvec2((bbox.minPt.x + bbox.maxPt.x) * 0.5, bbox.minPt.y);
					p2 = glm::dvec2((bbox.minPt.x + bbox.maxPt.x) * 0.5, bbox.maxPt.y);
				}

				p1 = layers[i].shapes[j]->worldCoordinate(p1);
				p2 = layers[i].shapes[j]->worldCoordinate(p2);

				// invert the y coordinate
				p1.y = 800 - p1.y;
				p2.y = 800 - p2.y;

				// add joints
				boost::shared_ptr<Joint> joint1 = boost::shared_ptr<Joint>(new PinJoint(p1));
				diagram.addJoint(joint1);
				boost::shared_ptr<Joint> joint2 = boost::shared_ptr<Joint>(new PinJoint(p2));
				diagram.addJoint(joint2);

				// add link
				boost::shared_ptr<Link> link = diagram.addLink(false, joint1, joint2);

				// add body
				std::vector<glm::dvec2> points = layers[i].shapes[j]->getPoints();
				for (int i = 0; i < points.size(); ++i) {
					points[i].y = 800 - points[i].y;
				}
				diagram.addBody(joint1, joint2, points);
			}

			tmp_diagrams.push_back(diagram);
		}

		std::vector<KinematicDiagram> initial_diagrams;
		for (int i = 0; i < tmp_diagrams.size(); ++i) {
			initial_diagrams.push_back(tmp_diagrams[i].clone());
		}

		// find the closest points of each rigid part between two layers
		for (int i = 0; i < tmp_diagrams[0].links.size(); ++i) {
			for (int j = i + 1; j < tmp_diagrams[0].links.size(); ++j) {
				std::vector<boost::shared_ptr<Link>> link1;
				link1.push_back(initial_diagrams[0].links[i]);
				link1.push_back(initial_diagrams[1].links[i]);

				std::vector<boost::shared_ptr<Link>> link2;
				link2.push_back(initial_diagrams[0].links[j]);
				link2.push_back(initial_diagrams[1].links[j]);

				// find the best k such that the distance between two parts is shortest
				double k, l;
				double length = findShortestDistanceBetweenLinks(link1[0], link2[0], link1[1], link2[1], k, l);

				if (length < 60) {
					for (int s = 0; s < 2; ++s) {
						glm::dvec2 p = link1[s]->joints[0]->pos + (link1[s]->joints[1]->pos - link1[s]->joints[0]->pos) * k;
						glm::dvec2 q = link2[s]->joints[0]->pos + (link2[s]->joints[1]->pos - link2[s]->joints[0]->pos) * l;

						if (length > 20) {
							// update the diagram
							boost::shared_ptr<Joint> joint1 = boost::shared_ptr<Joint>(new PinJoint(p));
							initial_diagrams[s].addJoint(joint1);
							boost::shared_ptr<Joint> joint2 = boost::shared_ptr<Joint>(new PinJoint(q));
							initial_diagrams[s].addJoint(joint2);

							initial_diagrams[s].setJointToLink(joint1, link1[s]);
							initial_diagrams[s].setJointToLink(joint2, link2[s]);

							boost::shared_ptr<Link> link3 = initial_diagrams[s].addLink(joint1, joint2);
						}
						else {
							// if the distance between two parts is too close, use a single joint to connect them
							p = (p + q) * 0.5;
							boost::shared_ptr<Joint> joint = boost::shared_ptr<Joint>(new PinJoint(p));
							initial_diagrams[s].addJoint(joint);
							initial_diagrams[s].setJointToLink(joint, link1[s]);
							initial_diagrams[s].setJointToLink(joint, link2[s]);
						}
					}
				}
			}
		}

		// check if the joint is attached to the ground
		for (int i = 0; i < initial_diagrams[0].joints.size(); ++i) {
			if (glm::length(initial_diagrams[0].joints[i]->pos - initial_diagrams[1].joints[i]->pos) <= 5) {
				for (int s = 0; s < 2; ++s) {
					initial_diagrams[s].joints[i]->ground = true;
				}
			}
		}

		// check if the link has only one ground-attached joint, which indicats that the link is a driver
		for (int s = 0; s < 2; ++s) {
			for (int l = 0; l < initial_diagrams[s].links.size(); ++l) {
				int ground_joint_count = 0;
				for (int j = 0; j < initial_diagrams[s].links[l]->joints.size(); ++j) {
					if (initial_diagrams[s].links[l]->joints[j]->ground) {
						ground_joint_count++;
					}
				}
				if (ground_joint_count == 1) {
					initial_diagrams[s].links[l]->driver = true;

					// We have to have only one driving link, so once we find one, we break.
					break;
				}
			}
		}

		// HACK???
		// remove the joint that has only one link
		/*
		for (int s = 0; s < 2; ++s) {
			for (int i = 0; i < initial_diagrams[s].joints.size(); ++i) {
				if (initial_diagrams[s].joints[i]->ground) continue;

				if (initial_diagrams[s].joints[i]->links.size() <= 1) {
					initial_diagrams[s].removeJoint(initial_diagrams[s].joints[i]);
				}
			}
		}
		*/

		return initial_diagrams;
	}

	double LinkageSolver::findShortestDistanceBetweenLinks(boost::shared_ptr<Link> link0a, boost::shared_ptr<Link> link0b, boost::shared_ptr<Link> link1a, boost::shared_ptr<Link> link1b, double& best_k, double& best_l) {
		// find the best k such that the distance between two parts is shortest
		double min_length = std::numeric_limits<double>::max();
		best_k = 0.0;
		best_l = 0.0;

		for (double k = 0.0; k <= 1.0; k += 0.01) {
			// find the best l such that the length between two parts is same across layers
			double min_diff = std::numeric_limits<double>::max();
			double best_l2 = 0.0;
			double best_length = 0.0;

			for (double l = 0.0; l <= 1.0; l += 0.01) {
				glm::dvec2 p0 = link0a->joints[0]->pos + (link0a->joints[1]->pos - link0a->joints[0]->pos) * k;
				glm::dvec2 q0 = link0b->joints[0]->pos + (link0b->joints[1]->pos - link0b->joints[0]->pos) * l;
				double length0 = glm::length(p0 - q0);

				glm::dvec2 p1 = link1a->joints[0]->pos + (link1a->joints[1]->pos - link1a->joints[0]->pos) * k;
				glm::dvec2 q1 = link1b->joints[0]->pos + (link1b->joints[1]->pos - link1b->joints[0]->pos) * l;
				double length1 = glm::length(p1 - q1);

				if (abs(length0 - length1) < min_diff) {
					min_diff = abs(length0 - length1);
					best_l2 = l;
					best_length = (length0 + length1) * 0.5;
				}
			}

			if (best_length < min_length) {
				min_length = best_length;
				best_k = k;
				best_l = best_l2;
			}
		}

		return min_length;
	}

	KinematicDiagram LinkageSolver::solve(std::vector<KinematicDiagram> initial_diagrams) {
		KinematicDiagram result_diagram = initial_diagrams[0].clone();

		boost::shared_ptr<TreeNode> root = constructTree(initial_diagrams);
		std::list<boost::shared_ptr<TreeNode>> queue;
		queue.push_back(root);
		while (!queue.empty()) {
			boost::shared_ptr<TreeNode> node = queue.front();
			queue.pop_front();

			if (node->hasGrandParent()) {
				// use the grand parent, parent, and this node to add a linkage
				std::vector<boost::shared_ptr<Joint>> p1 = node->parent->parent_joint;
				std::vector<boost::shared_ptr<Joint>> p2 = node->parent_joint;
				std::vector<boost::shared_ptr<Joint>> p3;
				for (int i = 0; i < node->links[0]->joints.size(); ++i) {
					if (node->links[0]->joints[i] == p2[0]) continue;

					p3.push_back(node->links[0]->joints[i]);
					p3.push_back(node->links[1]->joints[i]);
					break;
				}
				std::vector<boost::shared_ptr<Joint>> p0;
				for (int i = 0; i < node->parent->parent->links[0]->joints.size(); ++i) {
					if (node->parent->parent->links[0]->joints[i] == p1[0]) continue;

					p0.push_back(node->parent->parent->links[0]->joints[i]);
					p0.push_back(node->parent->parent->links[1]->joints[i]);
					break;
				}

				// convert the local coordinates
				std::vector<glm::dvec2> p0_pos(2);
				std::vector<glm::dvec2> p1_pos(2);
				std::vector<glm::dvec2> p2_pos(2);
				std::vector<glm::dvec2> p3_pos(2);
				for (int s = 0; s < 2; ++s) {
					p0_pos[s] = glm::dvec2(glm::inverse(node->parent->parent->mat[s]) * glm::dvec4(p0[s]->pos, 0, 1));
					p1_pos[s] = glm::dvec2(glm::inverse(node->parent->parent->mat[s]) * glm::dvec4(p1[s]->pos, 0, 1));
					p2_pos[s] = glm::dvec2(glm::inverse(node->parent->parent->mat[s]) * glm::dvec4(p2[s]->pos, 0, 1));
					p3_pos[s] = glm::dvec2(glm::inverse(node->parent->parent->mat[s]) * glm::dvec4(p3[s]->pos, 0, 1));
				}

				// calculate the bisector of ...
				//
				std::vector<glm::dvec2> v;
				for (int s = 0; s < 2; ++s) {
					glm::dvec2 dir = p3_pos[s] - p2_pos[s];
					v.push_back(dir / glm::length(dir));
				}

				double l = 40;
				if (node->links[0]->id == 2) {
					l = -40;
				}
				std::vector<glm::dvec2> pts2;
				for (int i = 0; i < v.size(); ++i) {
					pts2.push_back(p2_pos[i] + v[i] * l - p1_pos[i]);
				}

				glm::dvec2 perp = pts2[0] - pts2[1];
				perp = glm::dvec2(-perp.y, perp.x);
				glm::dvec2 c = (pts2[0] + pts2[1]) * 0.5;
				//glm::dvec2 prev_pt = glm::dvec2(glm::inverse(mat[0]) * glm::dvec4(input_points[0][pi - 1], 0, 1));
				glm::dvec2 pts1 = kinematics::lineLineIntersection(c, c + perp, glm::dvec2(0, 0), p0[0]->pos - p1[0]->pos);

				// add pts1 to link 0
				bool ground = false;
				if (node->parent->parent->links[0]->isGrounded()) ground = true;
				boost::shared_ptr<Joint> pts1_joint = boost::shared_ptr<Joint>(new PinJoint(ground, glm::dvec2(node->parent->parent->mat[0] * glm::dvec4(pts1 + p1_pos[0], 0, 1))));
				result_diagram.addJoint(pts1_joint);
				result_diagram.setJointToLink(pts1_joint, result_diagram.links[node->parent->parent->links[0]->id]);

				boost::shared_ptr<Joint> pts2_joint = boost::shared_ptr<Joint>(new PinJoint(glm::dvec2(node->parent->parent->mat[0] * glm::dvec4(pts2[0] + p1_pos[0], 0, 1))));
				result_diagram.addJoint(pts2_joint);
				result_diagram.setJointToLink(pts2_joint, result_diagram.links[node->links[0]->id]);

				result_diagram.addLink(pts1_joint, pts2_joint);
			}

			for (int i = 0; i < node->child_nodes.size(); ++i) {
				queue.push_back(node->child_nodes[i].second);
			}
		}

		result_diagram.initialize();

		return result_diagram;
	}

	boost::shared_ptr<TreeNode> LinkageSolver::constructTree(std::vector<KinematicDiagram> diagrams) {
		boost::shared_ptr<TreeNode> root = findGroundLink(diagrams);
		constructSubTree(root);
		return root;
	}

	void LinkageSolver::constructSubTree(boost::shared_ptr<TreeNode> parent) {
		for (int i = 0; i < parent->links[0]->joints.size(); ++i) {
			for (int j = 0; j < parent->links[0]->joints[i]->links.size(); ++j) {
				if (parent->links[0]->joints[i]->links[j] == parent->links[0]) continue;
				if (parent->parent && parent->links[0]->joints[i]->links[j] == parent->parent->links[0]) continue;

				boost::shared_ptr<TreeNode> child = boost::shared_ptr<TreeNode>(new TreeNode());
				child->parent = parent;
				std::vector<boost::shared_ptr<Joint>> joints;
				joints.push_back(parent->links[0]->joints[i]);
				joints.push_back(parent->links[1]->joints[i]);
				child->parent_joint = joints;
				child->links.push_back(parent->links[0]->joints[i]->links[j]);
				child->links.push_back(parent->links[1]->joints[i]->links[j]);
				parent->child_nodes.push_back(std::make_pair(joints, child));

				if (parent->parent) {
					for (int s = 0; s < 2; ++s) {
						glm::dvec2 v = joints[s]->pos - parent->parent_joint[s]->pos;
						double angle = atan2(-v.y, -v.x);

						parent->mat.push_back(glm::rotate(glm::translate(parent->parent->mat[s], glm::dvec3(v.x, v.y, 0)), angle, glm::dvec3(0, 0, 1)));
					}
				}
				else {
					parent->mat.push_back(glm::dmat4x4());
					parent->mat.push_back(glm::dmat4x4());
				}

				constructSubTree(child);
			}
		}
	}

	boost::shared_ptr<TreeNode> LinkageSolver::findGroundLink(std::vector<KinematicDiagram> diagrams) {
		for (int i = 0; i < diagrams[0].links.size(); ++i) {
			if (diagrams[0].links[i]->isGrounded()) {
				boost::shared_ptr<TreeNode> node = boost::shared_ptr<TreeNode>(new TreeNode());
				node->links.push_back(diagrams[0].links[i]);
				node->links.push_back(diagrams[1].links[i]);
				return node;
			}
		}

		throw "No ground link.";
	}

}