#include "LinkageSolver.h"
#include "PinJoint.h"
#include "Link.h"
#include "BoundingBox.h"
#include "Utils.h"
#include "Kinematics.h"
#include "Layer.h"
#include "Shape.h"

namespace kinematics {

	bool TreeNode::hasGrandParent() {
		if (!parent) return false;
		else if (!parent->parent) return false;
		else return true;
	}

	LinkageSolver::LinkageSolver() {
	}

	void LinkageSolver::adjustSketch(std::vector<canvas::Layer>& layers) {
		// check if the shape is attached to the ground
		int ground_shape_id = -1;
		for (int i = 0; i < layers[0].shapes.size(); ++i) {
			canvas::BoundingBox bbox0 = layers[0].shapes[i]->boundingBox();
			glm::dvec2 topLeft0 = bbox0.minPt;
			glm::dvec2 bottomRight0 = bbox0.maxPt;
			topLeft0 = layers[0].shapes[i]->worldCoordinate(topLeft0);
			bottomRight0 = layers[0].shapes[i]->worldCoordinate(bottomRight0);

			canvas::BoundingBox bbox1 = layers[1].shapes[i]->boundingBox();
			glm::dvec2 topLeft1 = bbox1.minPt;
			glm::dvec2 bottomRight1 = bbox1.maxPt;
			topLeft1 = layers[1].shapes[i]->worldCoordinate(topLeft1);
			bottomRight1 = layers[1].shapes[i]->worldCoordinate(bottomRight1);

			if (glm::length(topLeft0 - topLeft1) < 0.1 && glm::length(bottomRight0 - bottomRight1) < 0.1) {
				ground_shape_id = i;
				break;
			}
		}

		// construct a minimum spanning tree to determine the connectivity between parts
		SimpleGraph graph = constructGraph(layers);
		graph = graph.minimumSpanningTree();

		for (int i = 0; i < layers[0].shapes.size(); ++i) {
			for (int j = i + 1; j < layers[0].shapes.size(); ++j) {
				if (!graph.isNeighbor(i, j)) continue;

				if (graph.getEdge(i, j)->weight > 50) continue;

				std::vector<glm::dvec2> midPt(2);
				std::vector<glm::dvec2> pt_i(2);
				std::vector<glm::dvec2> pt_j(2);
				std::vector<double> ratio_i(2);
				std::vector<double> ratio_j(2);
				std::vector<glm::dvec2> v1(2), v2(2), v3(2), v4(2);
				for (int s = 0; s < 2; ++s) {
					canvas::BoundingBox bbox1 = layers[s].shapes[i]->boundingBox();
					canvas::BoundingBox bbox2 = layers[s].shapes[j]->boundingBox();

					if (bbox1.width() >= bbox1.height()) {
						v1[s] = glm::dvec2(bbox1.minPt.x, (bbox1.minPt.y + bbox1.maxPt.y) * 0.5);
						v2[s] = glm::dvec2(bbox1.maxPt.x, (bbox1.minPt.y + bbox1.maxPt.y) * 0.5);
					}
					else {
						v1[s] = glm::dvec2((bbox1.minPt.x + bbox1.maxPt.x) * 0.5, bbox1.minPt.y);
						v2[s] = glm::dvec2((bbox1.minPt.x + bbox1.maxPt.x) * 0.5, bbox1.maxPt.y);
					}
					v1[s] = layers[s].shapes[i]->worldCoordinate(v1[s]);
					v2[s] = layers[s].shapes[i]->worldCoordinate(v2[s]);
					if (bbox2.width() >= bbox2.height()) {
						v3[s] = glm::dvec2(bbox2.minPt.x, (bbox2.minPt.y + bbox2.maxPt.y) * 0.5);
						v4[s] = glm::dvec2(bbox2.maxPt.x, (bbox2.minPt.y + bbox2.maxPt.y) * 0.5);
					}
					else {
						v3[s] = glm::dvec2((bbox2.minPt.x + bbox2.maxPt.x) * 0.5, bbox2.minPt.y);
						v4[s] = glm::dvec2((bbox2.minPt.x + bbox2.maxPt.x) * 0.5, bbox2.maxPt.y);
					}
					v3[s] = layers[s].shapes[j]->worldCoordinate(v3[s]);
					v4[s] = layers[s].shapes[j]->worldCoordinate(v4[s]);

					glm::dvec2 p, q;
					segmentSegmentDistance(v1[s], v2[s], v3[s], v4[s], p, q);

					ratio_i[s] = glm::length(p - v1[s]) / glm::length(v2[s] - v1[s]);
					ratio_j[s] = glm::length(q - v3[s]) / glm::length(v4[s] - v3[s]);
				}

				double ratio_i_avg = (ratio_i[0] + ratio_i[1]) * 0.5;
				double ratio_j_avg = (ratio_j[0] + ratio_j[1]) * 0.5;


				// translate part 1
				std::vector<glm::dvec2> offset(2);
				for (int s = 0; s < 2; ++s) {
					glm::dvec2 joint_pos = ((v2[s] - v1[s]) * ratio_i_avg + v1[s] + (v4[s] - v3[s]) * ratio_j_avg + v3[s]) * 0.5;
					offset[s] = joint_pos - (v2[s] - v1[s]) * ratio_i_avg - v1[s];
				}
				QMap<int, bool> visited;
				visited[j] = true;
				translatePart(layers, i, offset, visited, graph);

				// translate part 1
				for (int s = 0; s < 2; ++s) {
					glm::dvec2 joint_pos = ((v2[s] - v1[s]) * ratio_i_avg + v1[s] + (v4[s] - v3[s]) * ratio_j_avg + v3[s]) * 0.5;
					offset[s] = joint_pos - (v4[s] - v3[s]) * ratio_j_avg - v3[s];
				}
				visited.clear();
				visited[i] = true;
				translatePart(layers, j, offset, visited, graph);

			}
		}

		// translate pose 2 such that the position of the grounded shape is exactly match with pose1
		glm::dvec2 offset = layers[0].shapes[ground_shape_id]->worldCoordinate(layers[0].shapes[ground_shape_id]->getCenter()) - layers[1].shapes[ground_shape_id]->worldCoordinate(layers[1].shapes[ground_shape_id]->getCenter());
		for (int i = 0; i < layers[1].shapes.size(); ++i) {
			layers[1].shapes[i]->translate(offset);
		}
		
	}

	void LinkageSolver::translatePart(std::vector<canvas::Layer>& layers, int shape_id, std::vector<glm::dvec2> offset, QMap<int, bool> visited, const SimpleGraph& mst) {
		for (int s = 0; s < layers.size(); ++s) {
			layers[s].shapes[shape_id]->translate(offset[s]);
		}
		visited[shape_id] = true;

		for (int i = 0; i < mst.vertices[shape_id]->edges.size(); ++i) {
			int edge_id = mst.vertices[shape_id]->edges[i];
			for (int j = 0; j < mst.edges[edge_id]->vertices.size(); ++j) {
				int v_id = mst.edges[edge_id]->vertices[j];
				if (visited.contains(v_id)) continue;

				translatePart(layers, v_id, offset, visited, mst);
			}
		}
	}

	std::vector<KinematicDiagram> LinkageSolver::initialKinematicDiagram(std::vector<canvas::Layer> layers) {
		SimpleGraph graph = constructGraph(layers);
		graph = graph.minimumSpanningTree();

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
				// don't connect the parts if they are not directory connected in Minimum Spanning Tree
				if (!graph.isNeighbor(i, j)) continue;

				std::vector<boost::shared_ptr<Link>> link1;
				link1.push_back(initial_diagrams[0].links[i]);
				link1.push_back(initial_diagrams[1].links[i]);

				std::vector<boost::shared_ptr<Link>> link2;
				link2.push_back(initial_diagrams[0].links[j]);
				link2.push_back(initial_diagrams[1].links[j]);

				if (graph.getEdge(i, j)->weight > 50) {
					// find the best k such that the distance between two parts is shortest
					double k, l;
					double length = findShortestDistanceBetweenLinks(link1[0], link2[0], link1[1], link2[1], k, l);

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
					}
				}
				else {
					for (int s = 0; s < 2; ++s) {
						glm::dvec2 p, q;
						segmentSegmentDistance(link1[s]->joints[0]->pos, link1[s]->joints[1]->pos, link2[s]->joints[0]->pos, link2[s]->joints[1]->pos, p, q);

						// use a single joint to connect parts
						boost::shared_ptr<Joint> joint;
						p = (p + q) * 0.5;
						if (glm::length(p - link1[s]->joints[0]->pos) < 0.1) {
							joint = link1[s]->joints[0];
							initial_diagrams[s].setJointToLink(joint, link2[s]);
						}
						else if (glm::length(p - link1[s]->joints[1]->pos) < 0.1) {
							joint = link1[s]->joints[1];
							initial_diagrams[s].setJointToLink(joint, link2[s]);
						}
						else if (glm::length(p - link2[s]->joints[0]->pos) < 0.1) {
							joint = link2[s]->joints[0];
							initial_diagrams[s].setJointToLink(joint, link1[s]);
						}
						else if (glm::length(p - link2[s]->joints[1]->pos) < 0.1) {
							joint = link2[s]->joints[1];
							initial_diagrams[s].setJointToLink(joint, link1[s]);
						}
						else {
							joint = boost::shared_ptr<Joint>(new PinJoint(p));
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
			if (glm::length(initial_diagrams[0].joints[i]->pos - initial_diagrams[1].joints[i]->pos) <= 0.1) {
				for (int s = 0; s < 2; ++s) {
					initial_diagrams[s].joints[i]->ground = true;
				}
			}
		}

		// check if the link has only one ground-attached joint, which indicats that the link is a driver
		std::vector<double> angles;
		for (int s = 0; s < 2; ++s) {
			for (int l = 0; l < initial_diagrams[s].links.size(); ++l) {
				int ground_joint_count = 0;
				boost::shared_ptr<Joint> ground_joint;
				for (int j = 0; j < initial_diagrams[s].links[l]->joints.size(); ++j) {
					if (initial_diagrams[s].links[l]->joints[j]->ground) {
						ground_joint_count++;
						ground_joint = initial_diagrams[s].links[l]->joints[j];
					}
				}

				if (ground_joint_count == 1) {
					initial_diagrams[s].links[l]->driver = true;
					
					// find another joint
					boost::shared_ptr<Joint> non_ground_joint;
					double max_dist = 0;
					for (int j = 0; j < initial_diagrams[s].links[l]->joints.size(); ++j) {
						if (initial_diagrams[s].links[l]->joints[j]->id == ground_joint->id) continue;

						double dist = glm::length(initial_diagrams[s].links[l]->joints[j]->pos - ground_joint->pos);
						if (dist > max_dist) {
							max_dist = dist;
							non_ground_joint = initial_diagrams[s].links[l]->joints[j];
						}
					}
					
					// calcuate angle
					glm::dvec2 vec = non_ground_joint->pos - ground_joint->pos;
					double angle = atan2(vec.y, vec.x);
					angles.push_back(angle);

					// We have to have only one driving link, so once we find one, we break.
					break;
				}
			}
		}

		// calculate the range of the angle for the driver
		initial_diagrams[0].driver_angle_min = angles[0];
		initial_diagrams[0].driver_angle_max = angles[0];
		initial_diagrams[1].driver_angle_min = angles[1];
		initial_diagrams[1].driver_angle_max = angles[1];

		// construct the parts adjacency
		// we need to const the parts adjacency at this moment by considering all the poses
		for (int s = 0; s < 2; ++s) {
			for (int i = 0; i < initial_diagrams[s].bodies.size(); ++i) {
				initial_diagrams[s].bodies[i]->neighbors.clear();
			}
		}
		for (int i = 0; i < initial_diagrams[0].bodies.size(); ++i) {
			for (int j = i + 1; j < initial_diagrams[0].bodies.size(); ++j) {
				bool adjacent = false;
				for (int s = 0; s < 2; ++s) {
					// check the adjacency between parts i and j
					if (polygonPolygonIntersection(initial_diagrams[s].bodies[i]->getActualPoints(), initial_diagrams[s].bodies[j]->getActualPoints())) {
						adjacent = true;
						break;
					}
				}

				if (adjacent) {
					for (int s = 0; s < 2; ++s) {
						initial_diagrams[s].bodies[i]->neighbors[j] = true;
						initial_diagrams[s].bodies[j]->neighbors[i] = true;
					}
				}
			}
		}

		return initial_diagrams;
	}

	SimpleGraph LinkageSolver::constructGraph(std::vector<canvas::Layer> layers) {
		SimpleGraph graph;
		for (int i = 0; i < layers[0].shapes.size(); ++i) {
			graph.addVertex(i);
		}

		for (int i = 0; i < layers[0].shapes.size(); ++i) {
			std::vector<glm::dvec2> p1(2);
			std::vector<glm::dvec2> p2(2);
			for (int s = 0; s < 2; ++s) {
				canvas::BoundingBox bbox1 = layers[s].shapes[i]->boundingBox();
				if (bbox1.width() >= bbox1.height()) {
					p1[s] = glm::dvec2(bbox1.minPt.x, (bbox1.minPt.y + bbox1.maxPt.y) * 0.5);
					p2[s] = glm::dvec2(bbox1.maxPt.x, (bbox1.minPt.y + bbox1.maxPt.y) * 0.5);
				}
				else {
					p1[s] = glm::dvec2((bbox1.minPt.x + bbox1.maxPt.x) * 0.5, bbox1.minPt.y);
					p2[s] = glm::dvec2((bbox1.minPt.x + bbox1.maxPt.x) * 0.5, bbox1.maxPt.y);
				}

				p1[s] = layers[s].shapes[i]->worldCoordinate(p1[s]);
				p2[s] = layers[s].shapes[i]->worldCoordinate(p2[s]);

				// invert the y coordinate
				p1[s].y = 800 - p1[s].y;
				p2[s].y = 800 - p2[s].y;
			}


			for (int j = i + 1; j < layers[0].shapes.size(); ++j) {
				std::vector<glm::dvec2> q1(2);
				std::vector<glm::dvec2> q2(2);

				// maximum distance between parts i and j
				double max_dist = 0;

				for (int s = 0; s < 2; ++s) {
					canvas::BoundingBox bbox2 = layers[s].shapes[j]->boundingBox();
					if (bbox2.width() >= bbox2.height()) {
						q1[s] = glm::dvec2(bbox2.minPt.x, (bbox2.minPt.y + bbox2.maxPt.y) * 0.5);
						q2[s] = glm::dvec2(bbox2.maxPt.x, (bbox2.minPt.y + bbox2.maxPt.y) * 0.5);
					}
					else {
						q1[s] = glm::dvec2((bbox2.minPt.x + bbox2.maxPt.x) * 0.5, bbox2.minPt.y);
						q2[s] = glm::dvec2((bbox2.minPt.x + bbox2.maxPt.x) * 0.5, bbox2.maxPt.y);
					}

					q1[s] = layers[s].shapes[j]->worldCoordinate(q1[s]);
					q2[s] = layers[s].shapes[j]->worldCoordinate(q2[s]);

					// invert the y coordinate
					q1[s].y = 800 - q1[s].y;
					q2[s].y = 800 - q2[s].y;

					double dist = segmentSegmentDistance(p1[s], p2[s], q1[s], q2[s]);
					if (dist > max_dist) {
						max_dist = dist;
					}
				}

				graph.addEdge(max_dist, i, j);
			}
		}

		return graph;
	}

	double LinkageSolver::findShortestDistanceBetweenLinks(boost::shared_ptr<Link> link0a, boost::shared_ptr<Link> link0b, boost::shared_ptr<Link> link1a, boost::shared_ptr<Link> link1b, double& best_k, double& best_l) {
		// find the best k such that the distance between two parts is shortest
		double min_length = std::numeric_limits<double>::max();
		best_k = 0.0;
		best_l = 0.0;

		double min_diff = std::numeric_limits<double>::max();

		const int N = 500;
		for (int ki = 0; ki <= N; ki++) {
			double k = (double)ki / N;

			for (int li = 0; li <= N; li++) {
				double l = (double)li / N;

				glm::dvec2 p0 = link0a->joints[0]->pos + (link0a->joints[1]->pos - link0a->joints[0]->pos) * k;
				glm::dvec2 q0 = link0b->joints[0]->pos + (link0b->joints[1]->pos - link0b->joints[0]->pos) * l;
				double length0 = glm::length(p0 - q0);

				glm::dvec2 p1 = link1a->joints[0]->pos + (link1a->joints[1]->pos - link1a->joints[0]->pos) * k;
				glm::dvec2 q1 = link1b->joints[0]->pos + (link1b->joints[1]->pos - link1b->joints[0]->pos) * l;
				double length1 = glm::length(p1 - q1);

				double avg_length = (length0 + length1) * 0.5;

				if (abs(length0 - length1) < min_diff + 0.01 && avg_length < min_length) {
					min_diff = abs(length0 - length1);
					best_k = k;
					best_l = l;
					min_length = avg_length;
				}
			}
		}

		return min_length;
	}

	KinematicDiagram LinkageSolver::solve(std::vector<KinematicDiagram> initial_diagrams, std::vector<double>& params) {
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

				// randomly pick a parameter value
				double l = 0;
				if (genRand() >= 0.5) {
					l = genRand(-80, -5);
				}
				else {
					l = genRand(5, 80);
				}
				//l = 20;
				params.push_back(l);

				// deterministically calculate the positions of pts1 and pts2
				std::vector<glm::dvec2> Q2;
				for (int i = 0; i < v.size(); ++i) {
					Q2.push_back(p2_pos[i] + v[i] * l - p1_pos[i]);
				}

				glm::dvec2 perp = Q2[0] - Q2[1];
				perp = glm::dvec2(-perp.y, perp.x);
				glm::dvec2 c = (Q2[0] + Q2[1]) * 0.5;
				//glm::dvec2 prev_pt = glm::dvec2(glm::inverse(mat[0]) * glm::dvec4(input_points[0][pi - 1], 0, 1));
				glm::dvec2 Q1 = kinematics::lineLineIntersection(c, c + perp, glm::dvec2(0, 0), p0[0]->pos - p1[0]->pos);
				glm::dvec2 Q1b = kinematics::lineLineIntersection(c, c + perp, glm::dvec2(0, 0), p0[1]->pos - p1[1]->pos);

				// add Q1 to link 0
				bool ground = false;
				if (node->parent->parent->links[0]->isGrounded()) ground = true;
				boost::shared_ptr<Joint> Q1_joint = boost::shared_ptr<Joint>(new PinJoint(ground, glm::dvec2(node->parent->parent->mat[0] * glm::dvec4(Q1 + p1_pos[0], 0, 1))));
				result_diagram.addJoint(Q1_joint);
				result_diagram.setJointToLink(Q1_joint, result_diagram.links[node->parent->parent->links[0]->id]);

				boost::shared_ptr<Joint> Q2_joint = boost::shared_ptr<Joint>(new PinJoint(glm::dvec2(node->parent->parent->mat[0] * glm::dvec4(Q2[0] + p1_pos[0], 0, 1))));
				result_diagram.addJoint(Q2_joint);
				result_diagram.setJointToLink(Q2_joint, result_diagram.links[node->links[0]->id]);

				result_diagram.addLink(Q1_joint, Q2_joint);

				params.push_back(glm::length(Q1));
			}

			for (int i = 0; i < node->child_nodes.size(); ++i) {
				queue.push_back(node->child_nodes[i].second);
			}
		}

		// set the angle range of the driver
		result_diagram.driver_angle_min = std::min(0.0, initial_diagrams[1].driver_angle_min - initial_diagrams[0].driver_angle_min);
		result_diagram.driver_angle_max = std::max(0.0, initial_diagrams[1].driver_angle_max - initial_diagrams[0].driver_angle_max);

		// initialize the link lengths
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

	KinematicDiagram LinkageSolver::optimize(std::vector<KinematicDiagram> initial_diagrams) {
		double min_length = std::numeric_limits<double>::max();
		KinematicDiagram best_diagram;

		for (int iter = 0; iter < 100; ++iter) {
			//std::cout << "iter: " << iter << std::endl;

			Kinematics kin;
			std::vector<double> params;
			kin.diagram = solve(initial_diagrams, params);
			//std::cout << "l: " << params[0] << std::endl;
			double time_step = 0.05;
			if (abs(kin.diagram.driver_angle_max - kin.diagram.driver_angle) < abs(kin.diagram.driver_angle_min - kin.diagram.driver_angle)) {
				time_step = -time_step;
			}

			try {
				KinematicDiagram backup_diagram = kin.diagram.clone();

				// do a kinematic simulation for collision detection
				while (kin.diagram.driver_angle + time_step >= kin.diagram.driver_angle_min && kin.diagram.driver_angle + time_step <= kin.diagram.driver_angle_max) {
					//std::cout << "angle: " << kin.diagram.driver_angle << std::endl;
					kin.stepForward(time_step);
				}

				double length = 0;
				for (int i = 0; i < params.size(); ++i) {
					length = std::max(length, abs(params[i]));
				}
				if (length < min_length) {
					//std::cout << "  best!!" << std::endl;
					min_length = length;
					best_diagram = backup_diagram.clone();
				}
			}
			catch (char* ex) {
			}
		}

		if (min_length < std::numeric_limits<double>::max()) {
			return best_diagram;
		}
		else {
			throw "Can't find valid linkage.";
		}
	}

}