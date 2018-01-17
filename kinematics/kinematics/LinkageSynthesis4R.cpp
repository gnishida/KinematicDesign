#include "LinkageSynthesis4R.h"
#include <opencv2/opencv.hpp>
#include "KinematicUtils.h"
#include "Kinematics.h"
#include "PinJoint.h"
#include "SliderHinge.h"
#include "BoundingBox.h"
#include "LeastSquareSolver.h"
#include "ZOrder.h"
#include "STLExporter.h"
#include "SCADExporter.h"
#include "Vertex.h"
#include "GLUtils.h"

namespace kinematics {

	LinkageSynthesis4R::LinkageSynthesis4R(const std::vector<Object25D>& fixed_bodies, const std::vector<std::pair<double, double>>& sigmas, bool avoid_branch_defect, double min_transmission_angle, double min_link_length, const std::vector<double>& weights) {
		this->fixed_bodies = fixed_bodies;
		this->sigmas = sigmas;
		this->avoid_branch_defect = avoid_branch_defect;
		this->min_transmission_angle = min_transmission_angle;
		this->min_link_length = min_link_length;
		this->weights = weights;
	}

	/**
	* Calculate solutions of 4R linkage given three poses.
	*
	* @param poses			three poses
	* @param solutions1	the output solutions for the world coordinates of the driving crank at the first pose, each of which contains a pair of the center point and the circle point
	* @param solutions2	the output solutions for the world coordinates of the follower at the first pose, each of which contains a pair of the center point and the circle point
	*/
	void LinkageSynthesis4R::calculateSolution(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, int num_samples, const Object25D& moving_body, std::vector<Solution>& solutions) {
		solutions.clear();

		srand(0);

		int cnt = 0;

		// calculate the bounding boxe of the valid regions
		BBox bbox = boundingBox(linkage_region_pts);

		for (int iter = 0; iter < num_samples && cnt < num_samples; iter++) {
			printf("\rsampling %d/%d", cnt, iter + 1);

			// perturbe the poses a little
			double position_error = 0.0;
			double orientation_error = 0.0;
			std::vector<glm::dmat3x3> perturbed_poses = perturbPoses(poses, sigmas, position_error, orientation_error);

			// sample joints within the linkage region
			std::vector<glm::dvec2> points(4);
			for (int i = 0; i < points.size(); i++) {
				while (true) {
					points[i] = glm::dvec2(genRand(bbox.minPt.x, bbox.maxPt.x), genRand(bbox.minPt.y, bbox.maxPt.y));
					if (withinPolygon(linkage_region_pts, points[i])) break;
				}
			}

			if (!optimizeCandidate(perturbed_poses, linkage_region_pts, bbox, points)) continue;

			// check hard constraints
			std::vector<std::vector<int>> zorder;
			if (!checkHardConstraints(points, perturbed_poses, linkage_region_pts, linkage_avoidance_pts, moving_body, zorder)) continue;
			solutions.push_back(Solution(0, points, position_error, orientation_error, perturbed_poses, zorder));
			cnt++;
		}
		printf("\n");
	}

	/**
	 * Optimize the linkage parameters based on the rigidity constraints.
	 * If it fails to optimize, return false.
	 */
	bool LinkageSynthesis4R::optimizeCandidate(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const BBox& bbox, std::vector<glm::dvec2>& points) {
		if (poses.size() == 2) {
			if (!optimizeLinkForTwoPoses(poses, linkage_region_pts, points[0], points[2])) return false;
			if (!optimizeLinkForTwoPoses(poses, linkage_region_pts, points[1], points[3])) return false;
		}
		else if (poses.size() == 3) {
			if (!optimizeLinkForThreePoses(poses, linkage_region_pts, points[0], points[2])) return false;
			if (!optimizeLinkForThreePoses(poses, linkage_region_pts, points[1], points[3])) return false;
		}
		else {
			if (!optimizeLink(poses, linkage_region_pts, bbox, points[0], points[2])) return false;
			if (!optimizeLink(poses, linkage_region_pts, bbox, points[1], points[3])) return false;
		}

		return true;
	}

	bool LinkageSynthesis4R::optimizeLink(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const BBox& bbox, glm::dvec2& A0, glm::dvec2& A1) {
		// setup the initial parameters for optimization
		column_vector starting_point(4);
		starting_point(0, 0) = A0.x;
		starting_point(1, 0) = A0.y;
		starting_point(2, 0) = A1.x;
		starting_point(3, 0) = A1.y;
		
		try {
			find_min(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7), SolverForLink(poses), SolverDerivForLink(poses), starting_point, -1);	

			A0.x = starting_point(0, 0);
			A0.y = starting_point(1, 0);
			A1.x = starting_point(2, 0);
			A1.y = starting_point(3, 0);

			// if the joints are outside the valid region, discard it.
			if (!withinPolygon(linkage_region_pts, A0)) return false;
			if (!withinPolygon(linkage_region_pts, A1)) return false;
		}
		catch (std::exception& e) {
			return false;
		}

		return true;
	}

	bool LinkageSynthesis4R::optimizeLinkForThreePoses(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, glm::dvec2& A0, glm::dvec2& A1) {
		// calculate the local coordinate of A1
		glm::dvec2 a = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(A1, 1));

		glm::dvec2 A2(poses[1] * glm::dvec3(a, 1));
		glm::dvec2 A3(poses[2] * glm::dvec3(a, 1));

		try {
			A0 = circleCenterFromThreePoints(A1, A2, A3);

			// if the center point is outside the valid region, discard it.
			if (!withinPolygon(linkage_region_pts, A0)) return false;

			// if the moving point is outside the valid region, discard it.
			//if (!withinPolygon(linkage_region_pts, A1)) return false;
		}
		catch (char* ex) {
			return false;
		}

		return true;
	}

	bool LinkageSynthesis4R::optimizeLinkForTwoPoses(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, glm::dvec2& A0, glm::dvec2& A1) {
		// calculate the local coordinate of A1
		glm::dvec2 a = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(A1, 1));

		glm::dvec2 A2(poses[1] * glm::dvec3(a, 1));

		glm::dvec2 M = (A1 + A2) * 0.5;
		glm::dvec2 v = A1 - A2;
		v /= glm::length(v);
		glm::dvec2 h(-v.y, v.x);

		A0 = M + h * glm::dot(A0 - M, h);

		// if the center point is outside the valid region, discard it.
		if (!withinPolygon(linkage_region_pts, A0)) return false;

		return true;
	}

	Solution LinkageSynthesis4R::findBestSolution(const std::vector<glm::dmat3x3>& poses, std::vector<Solution>& solutions, const std::vector<glm::dvec2>& linkage_region_pts, const cv::Mat& dist_map, const BBox& dist_map_bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, const Object25D& moving_body, int num_particles, int num_iterations, bool record_file) {
		// select the best solution based on the objective function
		if (solutions.size() > 0) {
			particleFilter(solutions, linkage_region_pts, dist_map, dist_map_bbox, linkage_avoidance_pts, moving_body, num_particles, num_iterations, record_file);
			return solutions[0];
		}
		else {
			return Solution(0, { { 0, 0 }, { 0, 2 }, { 2, 0 }, { 2, 2 } }, 0, 0, poses);
		}
	}

	double LinkageSynthesis4R::calculateCost(Solution& solution, const Object25D& moving_body, const cv::Mat& dist_map, const BBox& dist_map_bbox) {
		double dist = 0;
		for (int i = 0; i < solution.points.size(); i++) {
			dist += dist_map.at<double>(solution.points[i].y - dist_map_bbox.minPt.y, solution.points[i].x - dist_map_bbox.minPt.x);
		}
		double tortuosity = tortuosityOfTrajectory(solution.poses, solution.points, moving_body);
		std::vector<glm::dvec2> connected_pts;
		Kinematics kin = constructKinematics(solution.poses, solution.points, solution.zorder, moving_body, true, fixed_bodies, connected_pts);
		//double size = glm::length(solution.points[0] - solution.points[2]) + glm::length(solution.points[1] - solution.points[3]) + glm::length(solution.points[0] - connected_pts[0]) + glm::length(solution.points[1] - connected_pts[1]) + glm::length(solution.points[2] - connected_pts[2]) + glm::length(solution.points[3] - connected_pts[3]);
		double size = glm::length(solution.points[0] - solution.points[2]) + glm::length(solution.points[1] - solution.points[3]) + glm::length(solution.points[0] - connected_pts[0]) + glm::length(solution.points[1] - connected_pts[1]) + glm::length(solution.points[2] - connected_pts[2]) + glm::length(solution.points[3] - connected_pts[3]);
		int max_zorder = 0;
		for (int i = 0; i < solution.zorder.size(); i++) {
			for (int j = 0; j < solution.zorder[i].size(); j++) {
				max_zorder = std::max(max_zorder, solution.zorder[i][j]);
			}
		}

		return solution.position_error * weights[0] + solution.orientation_error * weights[1] + dist * weights[2] + tortuosity * weights[3] + size * weights[4] + max_zorder * weights[5];
	}

	/**
	* Construct a linkage.
	*/
	Kinematics LinkageSynthesis4R::constructKinematics(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points, const std::vector<std::vector<int>>& zorder, const Object25D& moving_body, bool connect_joints, const std::vector<Object25D>& fixed_bodies, std::vector<glm::dvec2>& connected_pts) {
		Kinematics kin;
		kin.linkage_type = 0;
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(0, true, points[0], zorder.size() == 3 ? zorder[2][0] : 1)));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(1, true, points[1], zorder.size() == 3 ? zorder[2][1] : 1)));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(2, false, points[2], zorder.size() == 3 ? zorder[2][0] : 1)));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(3, false, points[3], zorder.size() == 3 ? zorder[2][1] : 1)));
		kin.diagram.addLink(true, kin.diagram.joints[0], kin.diagram.joints[2], true, zorder.size() == 3 ? zorder[2][0] : 1);
		kin.diagram.addLink(false, kin.diagram.joints[1], kin.diagram.joints[3], true, zorder.size() == 3 ? zorder[2][1] : 1);
		kin.diagram.addLink(false, kin.diagram.joints[2], kin.diagram.joints[3], false);

		std::vector<Object25D> copied_fixed_bodies = fixed_bodies;

		// update the geometry
		updateMovingBodies(kin, moving_body);

		if (connect_joints) {
			kin.diagram.connectJointsToBodies(copied_fixed_bodies, zorder, connected_pts);
		}

		// add the fixed rigid bodies
		for (int i = 0; i < copied_fixed_bodies.size(); i++) {
			kin.diagram.addBody(kin.diagram.joints[0], kin.diagram.joints[1], copied_fixed_bodies[i]);
		}

		// calculte the range of motion
		std::pair<double, double> angle_range = checkRange(poses, points);
		kin.min_angle = angle_range.first;
		kin.max_angle = angle_range.second;

		return kin;
	}

	/**
	* update bodies.
	*/
	void LinkageSynthesis4R::updateMovingBodies(Kinematics& kin, const Object25D& moving_body) {
		kin.diagram.bodies.clear();
		kin.diagram.addBody(kin.diagram.joints[2], kin.diagram.joints[3], moving_body);
	}

	bool LinkageSynthesis4R::checkHardConstraints(std::vector<glm::dvec2>& points, const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, const Object25D& moving_body, std::vector<std::vector<int>>& zorder) {
		if (glm::length(points[0] - points[1]) < min_link_length) return false;
		if (glm::length(points[2] - points[3]) < min_link_length) return false;

		if (checkFolding(points)) return false;
		if (avoid_branch_defect && checkBranchDefect(poses, points)) return false;
		if (checkCircuitDefect(poses, points)) return false;
		if (checkOrderDefect(poses, points)) return false;

		// collision check for the main body
		// moving_body[0] means the main body without the joint connectors
		if (checkCollision(poses, points, fixed_bodies, moving_body[0], linkage_avoidance_pts)) return false;

		// record collision between connectors
		Kinematics kin = recordCollisionForConnectors(poses, points, fixed_bodies, moving_body);

		// determine the z-order of links and connectors
		try {
			zorder = ZOrder::zorderConnectors(kin.diagram.connectors);
		}
		catch (char* ex) {
			return false;
		}

		return true;
	}

	/**
	* Return the Grashof type.
	*
	* 0 -- Grashof (Drag-link)
	* 1 -- Grashof (Crank-rocker)
	* 2 -- Grashof (Rocker-crank)
	* 3 -- Grashof (Double-rocker)
	* 4 -- Non-Grashof (0-0 Rocker)
	* 5 -- Non-Grashof (pi-pi Rocker)
	* 6 -- Non-Grashof (pi-0 Rocker)
	* 7 -- Non-Grashof (0-pi Rocker)
	*/
	int LinkageSynthesis4R::getType(const std::vector<glm::dvec2>& points) {
		double g = glm::length(points[0] - points[1]);
		double a = glm::length(points[0] - points[2]);
		double b = glm::length(points[1] - points[3]);
		double h = glm::length(points[2] - points[3]);

		double T1 = g + h - a - b;
		double T2 = b + g - a - h;
		double T3 = b + h - a - g;

		if (T1 < 0 && T2 < 0 && T3 >= 0) {
			return 0;
		}
		else if (T1 >= 0 && T2 >= 0 && T3 >= 0) {
			return 1;
		}
		else if (T1 >= 0 && T2 < 0 && T3 < 0) {
			return 2;
		}
		else if (T1 < 0 && T2 >= 0 && T3 < 0) {
			return 3;
		}
		else if (T1 < 0 && T2 < 0 && T3 < 0) {
			return 4;
		}
		else if (T1 < 0 && T2 >= 0 && T3 >= 0) {
			return 5;
		}
		else if (T1 >= 0 && T2 < 0 && T3 >= 0) {
			return 6;
		}
		else if (T1 >= 0 && T2 >= 0 && T3 < 0) {
			return 7;
		}
		else {
			return -1;
		}
	}

	bool LinkageSynthesis4R::checkFolding(const std::vector<glm::dvec2>& points) {
		double g = glm::length(points[0] - points[1]);
		double a = glm::length(points[0] - points[2]);
		double b = glm::length(points[1] - points[3]);
		double h = glm::length(points[2] - points[3]);

		double T1 = g + h - a - b;
		double T2 = b + g - a - h;
		double T3 = b + h - a - g;

		if (abs(T1) < 0.05 || abs(T2) < 0.05 || abs(T3) < 0.05) return true;
		else return false;
	}

	std::pair<double, double> LinkageSynthesis4R::checkRange(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points) {
		if (poses.size() <= 2) return{ 0, 0 };

		glm::dvec2 dir1 = points[2] - points[0];
		glm::dvec2 dir2 = glm::dvec2(poses[1] * glm::inverse(poses[0]) * glm::dvec3(points[2], 1)) - points[0];
		glm::dvec2 dir3 = glm::dvec2(poses.back() * glm::inverse(poses[0]) * glm::dvec3(points[2], 1)) - points[0];

		double angle1 = std::atan2(dir1.y, dir1.x);
		double angle2 = std::atan2(dir2.y, dir2.x);
		double angle3 = std::atan2(dir3.y, dir3.x);

		// try clock-wise order
		double a1 = angle1;
		double a2 = angle2;
		double a3 = angle3;
		if (a2 > a1) {
			a2 -= M_PI * 2;
		}
		while (a3 > a2) {
			a3 -= M_PI * 2;
		}
		if (a1 - a3 < M_PI * 2) {
			return{ std::min(a1, a3), std::max(a1, a3) };
		}

		// try counter-clock-wise order
		if (angle2 < angle1) {
			angle2 += M_PI * 2;
		}
		if (angle3 < angle2) {
			angle3 += M_PI * 2;
		}
		if (angle3 - angle1 < M_PI * 2) {
			return{ std::min(angle1, angle3), std::max(angle1, angle3) };
		}

		return{ 0, 0 };
	}

	/**
	* Check if the linkage has order defect.
	* If there is an order defect, true is returned.
	* Otherwise, false is returned.
	*/
	bool LinkageSynthesis4R::checkOrderDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points) {
		glm::dvec2 inv_W = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(points[2], 1));

		double total_cw = 0;
		double total_ccw = 0;
		double prev = std::atan2(points[2].y - points[0].y, points[2].x - points[0].x);

		for (int i = 1; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving crank in the world coordinate system
			glm::dvec2 X = glm::dvec2(poses[i] * glm::dvec3(inv_W, 1));

			// calculate its angle
			double theta = std::atan2(X.y - points[0].y, X.x - points[0].x);

			if (theta >= prev) {
				total_cw += M_PI * 2 - theta + prev;
				total_ccw += theta - prev;
			}
			else {
				total_cw += prev - theta;
				total_ccw += M_PI * 2 - prev + theta;
			}

			prev = theta;
		}

		if (total_cw > M_PI * 2 && total_ccw > M_PI * 2) return true;
		else return false;
	}

	/**
	* Check if all the poses are in the same branch.
	* Drag-link and crank-rocker always do not have a branch defect.
	* For other types of linkage, the change in the sign of the angle between the coupler and the follower indicates the change of the branch.
	* If there is an branch defect, true is returned. Otherwise, false is returned.
	*
	* @param poses	pose matrices
	* @param p0		the world coordinates of the fixed point of the driving crank at the first pose
	* @param p1		the world coordinates of the fixed point of the follower at the first pose
	* @param p2		the world coordinates of the moving point of the driving crank at the first pose
	* @param p3		the world coordinates of the moving point of the follower at the first pose
	* @return		true if the branch defect is detected, false otherwise
	*/
	bool LinkageSynthesis4R::checkBranchDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points) {
		int type = getType(points);

		// drag-link and crank-rocker always do not have a branch defect
		if (type == 0 || type == 1) return false;

		int orig_sign = 1;

		// calculate the local coordinates of the circle points
		glm::dvec2 W1 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(points[2], 1));
		glm::dvec2 W2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(points[3], 1));

		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 X1 = glm::dvec2(poses[i] * glm::dvec3(W1, 1));
			glm::dvec2 X2 = glm::dvec2(poses[i] * glm::dvec3(W2, 1));

			// calculate its sign
			if (i == 0) {
				orig_sign = crossProduct(X2 - points[1], X1 - X2) >= 0 ? 1 : -1;
			}
			else {
				int sign = crossProduct(X2 - points[1], X1 - X2) >= 0 ? 1 : -1;
				if (sign != orig_sign) {
					return true;
				}
			}
		}

		return false;
	}

	bool LinkageSynthesis4R::checkCircuitDefect(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points) {
		int type = getType(points);

		// Non-grashof type does not have a circuit defect
		if (type >= 4) return false;

		int orig_sign0 = 1;
		int orig_sign1 = 1;
		int orig_sign2 = 1;
		int orig_sign3 = 1;

		// calculate the local coordinates of the circle points
		glm::dvec2 W1 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(points[2], 1));
		glm::dvec2 W2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(points[3], 1));

		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 X1 = glm::dvec2(poses[i] * glm::dvec3(W1, 1));
			glm::dvec2 X2 = glm::dvec2(poses[i] * glm::dvec3(W2, 1));

			// calculate its sign
			if (i == 0) {
				orig_sign0 = crossProduct(points[0] - points[1], X1 - points[0]) >= 0 ? 1 : -1;
				orig_sign1 = crossProduct(points[1] - X2, points[0] - points[1]) >= 0 ? 1 : -1;
				orig_sign2 = crossProduct(X1 - points[0], X2 - X1) >= 0 ? 1 : -1;
				orig_sign3 = crossProduct(X2 - X1, points[1] - X2) >= 0 ? 1 : -1;
			}
			else {
				int sign0 = crossProduct(points[0] - points[1], X1 - points[0]) >= 0 ? 1 : -1;
				int sign1 = crossProduct(points[1] - X2, points[0] - points[1]) >= 0 ? 1 : -1;
				int sign2 = crossProduct(X1 - points[0], X2 - X1) >= 0 ? 1 : -1;
				int sign3 = crossProduct(X2 - X1, points[1] - X2) >= 0 ? 1 : -1;

				if (type == 0) {
					if (sign2 != orig_sign2 || sign3 != orig_sign3) {
						return true;
					}
				}
				else if (type == 1) {
					if (sign1 != orig_sign1 || sign3 != orig_sign3) {
						return true;
					}
				}
				else if (type == 2) {
					if (sign0 != orig_sign0 || sign2 != orig_sign2) {
						return true;
					}
				}
				else if (type == 3) {
					if (sign0 != orig_sign0 || sign1 != orig_sign1) {
						return true;
					}
				}
			}
		}

		return false;
	}

	/**
	 * Check collision. If there are any collisions, return true.
	 * Only the main body is checked for collision.
	 *
	 * @param poses					N poses
	 * @param points				joint coordinates
	 * @param fixed_bodies			list of fixed bodies
	 * @param moving_body			moving body
	 * @param linkage_avoidance_pts	region to avoid for the linkage
	 * @return						true if collision occurs
	 */
	bool LinkageSynthesis4R::checkCollision(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points, const std::vector<Object25D>& fixed_bodies, const Object25D& moving_body, const std::vector<glm::dvec2>& linkage_avoidance_pts) {
		std::vector<glm::dvec2> connector_pts;
		kinematics::Kinematics kinematics = constructKinematics(poses, points, {}, moving_body, false, fixed_bodies, connector_pts);
		kinematics.diagram.initialize();

		// check the transmission angle
		if (avoid_branch_defect) {
			glm::dvec2 a = kinematics.diagram.joints[2]->pos - kinematics.diagram.joints[3]->pos;
			glm::dvec2 b = kinematics.diagram.joints[3]->pos - kinematics.diagram.joints[1]->pos;
			double t_angle = std::acos(glm::dot(a, b) / glm::length(a) / glm::length(b));
			if (std::abs(t_angle) < min_transmission_angle) return true;
		}

		// calculate the rotational angle of the driving crank for 1st, 2nd, and last poses
		// i.e., angles[0] = first pose, angles[1] = second pose, angles[2] = last pose
		std::vector<double> angles(3);
		glm::dvec2 w(glm::inverse(poses[0]) * glm::dvec3(points[2], 1));
		for (int i = 0; i < 2; i++) {
			glm::dvec2 W = glm::dvec2(poses[i] * glm::dvec3(w, 1));
			angles[i] = atan2(W.y - points[0].y, W.x - points[0].x);
		}
		{
			glm::dvec2 W = glm::dvec2(poses.back() * glm::dvec3(w, 1));
			angles[2] = atan2(W.y - points[0].y, W.x - points[0].x);
		}

		// order the angles based on their signs
		int type = 0;
		if (angles[0] < 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[1]) {
			type = 1;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[2]) {
			type = 2;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] >= angles[2]) {
			type = 3;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] >= 0 && (poses.size() >= 3 && angles[1] >= angles[2] || poses.size() == 2 && angles[1] - angles[0] > M_PI)) {
			type = 4;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] < 0 && (poses.size() >= 3 && angles[1] < angles[2] || poses.size() == 2 && angles[0] - angles[1] > M_PI)) {
			type = 5;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] < angles[2]) {
			type = 6;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[2]) {
			type = 7;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[1]) {
			type = 8;
			angles[2] += M_PI * 2;
		}

		if (angles[2] < angles[0]) {
			kinematics.invertSpeed();
		}

		// initialize the visited flag
		std::vector<bool> visited(angles.size(), false);
		visited[0] = true;
		int unvisited = 2;

		// run forward until collision is deteted or all the poses are reached
		while (true) {
			try {
				kinematics.stepForward(2, false);	// check only the main body for collision
			}
			catch (char* ex) {
				// if only some of the poses are reached before collision, the collision is detected.
				kinematics.clear();
				return true;
			}

			// check if the joints are within the linkage avoidance region
			for (int i = 0; i < kinematics.diagram.joints.size(); i++) {
				if (withinPolygon(linkage_avoidance_pts, kinematics.diagram.joints[i]->pos)) return true;
			}

			// check the transmission angle
			if (avoid_branch_defect) {
				glm::dvec2 a = kinematics.diagram.joints[2]->pos - kinematics.diagram.joints[3]->pos;
				glm::dvec2 b = kinematics.diagram.joints[3]->pos - kinematics.diagram.joints[1]->pos;
				double t_angle = std::acos(glm::dot(a, b) / glm::length(a) / glm::length(b));
				if (std::abs(t_angle) < min_transmission_angle) return true;
			}

			// calculate the angle of the driving crank
			double angle = atan2(kinematics.diagram.joints[2]->pos.y - points[0].y, kinematics.diagram.joints[2]->pos.x - points[0].x);

			// convert the sign of the angle
			if (type == 1 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 2 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 3 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 4 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 5 && angle < 0) {
				angle += M_PI * 2;
			}
			else if (type == 6 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 7 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 8 && angle < 0) {
				angle += M_PI * 2;
			}

			// check if the poses are reached
			for (int i = 0; i < angles.size(); i++) {
				if (visited[i]) continue;

				if (angles[2] >= angles[0]) {
					if (angle >= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
				else {
					if (angle <= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
			}

			// if all the poses are reached without collision, no collision is detected.
			if (unvisited == 0) {
				kinematics.clear();
				return false;
			}
		}

		kinematics.clear();
		return false;
	}

	Kinematics LinkageSynthesis4R::recordCollisionForConnectors(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points, const std::vector<Object25D> fixed_bodies, const Object25D& moving_body) {
		std::vector<glm::dvec2> connector_pts;
		Kinematics kinematics = constructKinematics(poses, points, {}, moving_body, true, fixed_bodies, connector_pts);
		kinematics.diagram.initialize();

		// calculate the rotational angle of the driving crank for 1st, 2nd, and last poses
		// i.e., angles[0] = first pose, angles[1] = second pose, angles[2] = last pose
		std::vector<double> angles(3);
		glm::dvec2 w(glm::inverse(poses[0]) * glm::dvec3(points[2], 1));
		for (int i = 0; i < 2; i++) {
			glm::dvec2 W = glm::dvec2(poses[i] * glm::dvec3(w, 1));
			angles[i] = atan2(W.y - points[0].y, W.x - points[0].x);
		}
		{
			glm::dvec2 W = glm::dvec2(poses.back() * glm::dvec3(w, 1));
			angles[2] = atan2(W.y - points[0].y, W.x - points[0].x);
		}

		// order the angles based on their signs
		int type = 0;
		if (angles[0] < 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[1]) {
			type = 1;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[2]) {
			type = 2;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] >= angles[2]) {
			type = 3;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] >= 0 && (poses.size() >= 3 && angles[1] >= angles[2] || poses.size() == 2 && angles[1] - angles[0] > M_PI)) {
			type = 4;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] < 0 && (poses.size() >= 3 && angles[1] < angles[2] || poses.size() == 2 && angles[0] - angles[1] > M_PI)) {
			type = 5;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] < angles[2]) {
			type = 6;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[2]) {
			type = 7;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[1]) {
			type = 8;
			angles[2] += M_PI * 2;
		}

		if (angles[2] < angles[0]) {
			kinematics.invertSpeed();
		}

		// initialize the visited flag
		std::vector<bool> visited(angles.size(), false);
		visited[0] = true;
		int unvisited = 2;

		// check the collision at the initial pose
		kinematics.diagram.recordCollisionForConnectors();

		// run forward until collision is deteted or all the poses are reached
		while (true) {
			try {
				kinematics.stepForward(3, false);
			}
			catch (char* ex) {
				// if only some of the poses are reached before collision, the collision is detected.
				return kinematics;
			}

			// calculate the angle of the driving crank
			double angle = atan2(kinematics.diagram.joints[2]->pos.y - points[0].y, kinematics.diagram.joints[2]->pos.x - points[0].x);

			// convert the sign of the angle
			if (type == 1 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 2 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 3 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 4 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 5 && angle < 0) {
				angle += M_PI * 2;
			}
			else if (type == 6 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 7 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 8 && angle < 0) {
				angle += M_PI * 2;
			}

			// check if the poses are reached
			for (int i = 0; i < angles.size(); i++) {
				if (visited[i]) continue;

				if (angles[2] >= angles[0]) {
					if (angle >= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
				else {
					if (angle <= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
			}

			// if all the poses are reached without collision, no collision is detected.
			if (unvisited == 0) {
				return kinematics;
			}
		}

		return kinematics;
	}

	double LinkageSynthesis4R::tortuosityOfTrajectory(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& points, const Object25D& moving_body) {
		// calculate the local coordinates of the body points
		glm::dmat3x3 inv_pose0 = glm::inverse(poses[0]);
		std::vector<glm::dvec2> body_pts_local(moving_body.polygons[0].points.size());
		for (int i = 0; i < moving_body.polygons[0].points.size(); i++) {
			body_pts_local[i] = glm::dvec2(inv_pose0 * glm::dvec3(moving_body.polygons[0].points[i], 1));
		}

		// calculate the length of the motion using straight lines between poses
		double length_of_straight = 0.0;
		std::vector<glm::dvec2> prev_body_pts = moving_body.polygons[0].points;
		for (int i = 1; i < poses.size(); i++) {
			std::vector<glm::dvec2> next_body_pts(moving_body.polygons[0].points.size());
			for (int k = 0; k < moving_body.polygons[0].points.size(); k++) {
				next_body_pts[k] = glm::dvec2(poses[i] * glm::dvec3(body_pts_local[k], 1));
				length_of_straight += glm::length(next_body_pts[k] - prev_body_pts[k]);
			}
			prev_body_pts = next_body_pts;
		}

		// create a kinematics
		std::vector<glm::dvec2> connector_pts;
		kinematics::Kinematics kinematics = constructKinematics(poses, points, {}, moving_body, false, {}, connector_pts);
		kinematics.diagram.initialize();

		// initialize the trajectory of the moving body
		prev_body_pts = moving_body.polygons[0].points;
		double length_of_trajectory = 0.0;

		// calculate the rotational angle of the driving crank for 1st, 2nd, and last poses
		// i.e., angles[0] = first pose, angles[1] = second pose, angles[2] = last pose
		std::vector<double> angles(3);
		glm::dvec2 w(glm::inverse(poses[0]) * glm::dvec3(points[2], 1));
		for (int i = 0; i < 2; i++) {
			glm::dvec2 W = glm::dvec2(poses[i] * glm::dvec3(w, 1));
			angles[i] = atan2(W.y - points[0].y, W.x - points[0].x);
		}
		{
			glm::dvec2 W = glm::dvec2(poses.back() * glm::dvec3(w, 1));
			angles[2] = atan2(W.y - points[0].y, W.x - points[0].x);
		}

		// order the angles based on their signs
		int type = 0;
		if (angles[0] < 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[1]) {
			type = 1;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[2]) {
			type = 2;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] >= angles[2]) {
			type = 3;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] >= 0 && (poses.size() >= 3 && angles[1] >= angles[2] || poses.size() == 2 && angles[1] - angles[0] > M_PI)) {
			type = 4;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] < 0 && (poses.size() >= 3 && angles[1] < angles[2] || poses.size() == 2 && angles[0] - angles[1] > M_PI)) {
			type = 5;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] < angles[2]) {
			type = 6;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[2]) {
			type = 7;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[1]) {
			type = 8;
			angles[2] += M_PI * 2;
		}

		if (angles[2] < angles[0]) {
			kinematics.invertSpeed();
		}

		// initialize the visited flag
		std::vector<bool> visited(angles.size(), false);
		visited[0] = true;
		int unvisited = 2;

		// run forward until collision is deteted or all the poses are reached
		while (true) {
			try {
				kinematics.stepForward(0, false);	// no collision check
			}
			catch (char* ex) {
				// if only some of the poses are reached before collision, the collision is detected.
				kinematics.clear();
				return length_of_trajectory / length_of_straight;
			}

			// calculate the angle of the driving crank
			double angle = atan2(kinematics.diagram.joints[2]->pos.y - points[0].y, kinematics.diagram.joints[2]->pos.x - points[0].x);

			// update the lengths of the trajectory of the moving body
			std::vector<glm::dvec2> next_body_pts = kinematics.diagram.bodies[0]->getActualPoints()[0];
			for (int i = 0; i < next_body_pts.size(); i++) {
				double length = glm::length(next_body_pts[i] - prev_body_pts[i]);
				length_of_trajectory += length;
				prev_body_pts[i] = next_body_pts[i];
			}

			// convert the sign of the angle
			if (type == 1 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 2 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 3 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 4 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 5 && angle < 0) {
				angle += M_PI * 2;
			}
			else if (type == 6 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 7 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 8 && angle < 0) {
				angle += M_PI * 2;
			}

			// check if the poses are reached
			for (int i = 0; i < angles.size(); i++) {
				if (visited[i]) continue;

				if (angles[2] >= angles[0]) {
					if (angle >= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
				else {
					if (angle <= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
			}

			// if all the poses are reached without collision, no collision is detected.
			if (unvisited == 0) {
				kinematics.clear();
				return length_of_trajectory / length_of_straight;
			}
		}

		kinematics.clear();
		return length_of_trajectory / length_of_straight;
	}

	void LinkageSynthesis4R::generate3DGeometry(const Kinematics& kinematics, std::vector<Vertex>& vertices) {
		// generate geometry of rigid bodies
		for (int j = 0; j < kinematics.diagram.bodies.size(); j++) {
			for (int k = 0; k < kinematics.diagram.bodies[j]->size(); k++) {
				std::vector<glm::dvec2> points = kinematics.diagram.bodies[j]->getActualPoints(k);
				std::vector<glm::dvec2> points2 = kinematics.diagram.bodies[j]->getActualPoints2(k);
				float z = kinematics.diagram.bodies[j]->polygons[k].depth1;
				float depth = kinematics.diagram.bodies[j]->polygons[k].depth2 - kinematics.diagram.bodies[j]->polygons[k].depth1;
				glutils::drawPrism(points, points2, depth, glm::vec4(0.7, 1, 0.7, 1), glm::translate(glm::mat4(), glm::vec3(0, 0, z)), vertices);
			}
		}

		// generate geometry of links
		for (int j = 0; j < kinematics.diagram.links.size(); j++) {
			// For the coupler, we can use the moving body itself as a coupler, 
			// so we do not need to create a coupler link.
			if (!kinematics.diagram.links[j]->actual_link) continue;

			glm::dvec2& p1 = kinematics.diagram.links[j]->joints[0]->pos;
			glm::dvec2& p2 = kinematics.diagram.links[j]->joints[1]->pos;
			std::vector<glm::dvec2> pts = generateRoundedBarPolygon(p1, p2, options->link_width / 2);
			float z = kinematics.diagram.links[j]->z * (options->link_depth + options->gap * 2 + options->joint_cap_depth) - options->link_depth;
			glutils::drawPrism(pts, options->link_depth, glm::vec4(0.7, 0.7, 0.7, 1), glm::translate(glm::mat4(), glm::vec3(0, 0, z)), vertices);
			glutils::drawPrism(pts, options->link_depth, glm::vec4(0.7, 0.7, 0.7, 1), glm::translate(glm::mat4(), glm::vec3(0, 0, -options->body_depth - z - options->link_depth)), vertices);
		}
	}

	void LinkageSynthesis4R::saveSTL(const QString& dirname, const std::vector<Kinematics>& kinematics) {
		for (int i = 0; i < kinematics.size(); i++) {
			// generate geometry of rigid bodies
			for (int j = 0; j < kinematics[i].diagram.bodies.size(); j++) {
				std::vector<Vertex> vertices;

				int N = kinematics[i].diagram.bodies[j]->size();
				for (int k = 0; k < N; k++) {
					std::vector<glm::dvec2> pts = kinematics[i].diagram.bodies[j]->getActualPoints(k);
					std::vector<glm::dvec2> pts2 = kinematics[i].diagram.bodies[j]->getActualPoints2(k);
					float z = kinematics[i].diagram.bodies[j]->polygons[k].depth1;
					float depth = kinematics[i].diagram.bodies[j]->polygons[k].depth2 - kinematics[i].diagram.bodies[j]->polygons[k].depth1;
					glutils::drawPrism(pts, pts2, depth, glm::vec4(0.7, 1, 0.7, 1), glm::translate(glm::mat4(), glm::vec3(0, 0, z)), vertices);
				}

				QString name = QString("body_%1_%2").arg(i).arg(j);
				QString filename = dirname + "/" + name + ".stl";
				STLExporter::save(filename, name, vertices);
			}

			// generate geometry of links
			for (int j = 0; j < kinematics[i].diagram.links.size(); j++) {
				// For the coupler, we can use the moving body itself as a coupler, 
				// so we do not need to create a coupler link.
				if (!kinematics[i].diagram.links[j]->actual_link) continue;

				glm::dvec2& p1 = kinematics[i].diagram.links[j]->joints[0]->pos;
				glm::dvec2& p2 = kinematics[i].diagram.links[j]->joints[1]->pos;
				std::vector<glm::dvec2> pts = generateRoundedBarPolygon(glm::vec2(), p2 - p1, options->link_width / 2);
				std::vector<std::vector<glm::dvec2>> holes(2);
				holes[0] = generateCirclePolygon(glm::vec2(), options->hole_radius);
				holes[1] = generateCirclePolygon(p2 - p1, options->hole_radius);

				std::vector<Vertex> vertices;
				glutils::drawPrismWithHoles(pts, holes, options->link_depth, glm::vec4(0.7, 0.7, 0.7, 1), glm::mat4(), vertices);

				QString name = QString("link_%1_%2").arg(i).arg(j);
				QString filename = dirname + "/" + name + ".stl";
				STLExporter::save(filename, name, vertices);
			}
		}
	}

	void LinkageSynthesis4R::saveSCAD(const QString& dirname, int index, const Kinematics& kinematics) {
		// generate geometry of rigid bodies
		for (int j = 0; j < kinematics.diagram.bodies.size(); j++) {
			QString name = QString("body_%1_%2").arg(index).arg(j);
			QString filename = dirname + "/" + name + ".scad";
			SCADExporter::save(filename, name, kinematics.diagram.bodies[j]);
		}

		// generate geometry of links
		for (int j = 0; j < kinematics.diagram.links.size(); j++) {
			// For the coupler, we can use the moving body itself as a coupler, 
			// so we do not need to create a coupler link.
			if (!kinematics.diagram.links[j]->actual_link) continue;

			glm::dvec2& p1 = kinematics.diagram.links[j]->joints[0]->pos;
			glm::dvec2& p2 = kinematics.diagram.links[j]->joints[1]->pos;
			std::vector<glm::dvec2> pts = generateRoundedBarPolygon(glm::vec2(), p2 - p1, options->link_width / 2);
			std::vector<std::vector<glm::dvec2>> holes(2);
			holes[0] = generateCirclePolygon(glm::vec2(), options->hole_radius);
			holes[1] = generateCirclePolygon(p2 - p1, options->hole_radius);

			QString name = QString("link_%1_%2").arg(index).arg(j);
			QString filename = dirname + "/" + name + ".scad";
			SCADExporter::save(filename, name, pts, holes, options->link_depth);
		}
	}

}