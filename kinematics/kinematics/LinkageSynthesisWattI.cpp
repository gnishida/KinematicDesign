#include "LinkageSynthesisWattI.h"
#include "KinematicUtils.h"
#include "Kinematics.h"
#include "PinJoint.h"
#include "SliderHinge.h"
#include "BoundingBox.h"
#include "LeastSquareSolver.h"
#include <opencv2/opencv.hpp>

namespace kinematics {

	/**
	* Calculate solutions of Watt I.
	*
	* @param poses			three poses
	* @param solutions1	the output solutions for the world coordinates of the driving crank at the first pose, each of which contains a pair of the center point and the circle point
	* @param solutions2	the output solutions for the world coordinates of the follower at the first pose, each of which contains a pair of the center point and the circle point
	*/
	void LinkageSynthesisWattI::calculateSolution(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, int num_samples, std::vector<Object25D>& fixed_body_pts, const std::vector<Object25D>& body_pts, std::vector<std::pair<double, double>>& sigmas, bool rotatable_crank, bool avoid_branch_defect, double min_link_length, std::vector<Solution>& solutions) {
		solutions.clear();

		srand(0);

		// calculate the center of the valid regions
		BBox bbox_world = boundingBox(linkage_region_pts);
		glm::dvec2 bbox_world_center = bbox_world.center();

		// convert the coordinates of the regions to the local coordinate system of the first pose
		std::vector<std::vector<glm::dvec2>> region_local(poses.size(), std::vector<glm::dvec2>(linkage_region_pts.size()));
		for (int i = 0; i < poses.size(); i++) {
			glm::dmat3x3 inv_pose0 = glm::inverse(poses[i][0]);

			for (int j = 0; j < linkage_region_pts.size(); j++) {
				region_local[i][j] = glm::dvec2(inv_pose0 * glm::dvec3(linkage_region_pts[j], 1));
			}
		}

		// calculate the bounding boxes of the regions
		std::vector<BBox> bbox_local(poses.size());
		for (int i = 0; i < poses.size(); i++) {
			bbox_local[i] = boundingBox(region_local[i]);
		}




		int cnt = 0;

		/*
		// calculate the enlarged linkage region for the sampling region
		std::vector<glm::dvec2> enlarged_linkage_region_pts;
		for (int i = 0; i < linkage_region_pts.size(); i++) {
			enlarged_linkage_region_pts.push_back((linkage_region_pts[i] - bbox_world_center) * (double)scale + bbox_world_center);
		}
		*/

		/*
		// convert the coordinates of the enlarged regions to the local coordinate system of the first pose
		std::vector<glm::dvec2> enlarged_region_local(enlarged_linkage_region_pts.size());
		for (int i = 0; i < enlarged_linkage_region_pts.size(); i++) {
			enlarged_region_local[i] = glm::dvec2(inv_pose0 * glm::dvec3(enlarged_linkage_region_pts[i], 1));
		}

		// calculate the bounding boxes of the enlarged regions
		BBox enlarged_bbox_local = boundingBox(enlarged_region_local);
		*/

		for (int iter = 0; iter < num_samples * 100 && cnt < num_samples; iter++) {
			printf("\rsampling %d/%d", cnt, iter + 1);

			// sample a linkage
			std::vector<glm::dvec2> points;
			if (!sampleLink(poses, linkage_region_pts, region_local, bbox_world, bbox_local, points)) continue;

			if (check(poses, points) > 1.0) continue;

			// check hard constraints
			//if (glm::length(A0 - B0) < min_link_length) continue;
			//if (glm::length(A1 - B1) < min_link_length) continue;

			//if (checkFolding(points)) continue;
			//if (rotatable_crank && checkRotatableCrankDefect(points)) continue;
			if (avoid_branch_defect && checkBranchDefect(poses, points)) continue;
			if (checkCircuitDefect(poses, points)) continue;
			if (checkOrderDefect(poses, points)) continue;

			// collision check
			if (checkCollision(poses, points, fixed_body_pts, body_pts, 2)) continue;

			// determine the z-order of links and connectors
			std::vector<std::vector<int>> zorder;
			/*try {
				zorder = ZOrder::zorderConnectors(kin.diagram.connectors);
			}
			catch (char* ex) {
				continue;
			}*/

			solutions.push_back(Solution(points, 0, 0, 0, poses, zorder));

			check(poses, points);
			cnt++;
		}

		printf("\n");

	}

	/**
	 * @param linkage_region_pts_local	linkage region in the local coordinate system of moving objects (i = 0, 1)
	 * @param bbox_local				bounding box in the local coordinate system of moving objects (i = 0, 1)
	 */
	bool LinkageSynthesisWattI::sampleLink(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<std::vector<glm::dvec2>>& linkage_region_pts_local, const BBox& bbox_world, const std::vector<BBox>& bbox_local, std::vector<glm::dvec2>& points) {
		points.resize(7);

		// sample P0 within the valid region
		points[0] = glm::dvec2(genRand(bbox_world.minPt.x, bbox_world.maxPt.x), genRand(bbox_world.minPt.y, bbox_world.maxPt.y));

		// if the sampled point is outside the valid region, discard it.
		if (!withinPolygon(linkage_region_pts, points[0])) return false;
		
		// sample P1 within the valid region as the local coordinate of a circle point
		points[1] = glm::dvec2(genRand(bbox_local[0].minPt.x, bbox_local[0].maxPt.x), genRand(bbox_local[0].minPt.y, bbox_local[0].maxPt.y));

		// if the sampled point is outside the valid region, discard it.
		if (!withinPolygon(linkage_region_pts_local[0], points[1])) return false;

		// sample P3 within the valid region as the local coordinate of a circle point
		points[3] = glm::dvec2(genRand(bbox_local[0].minPt.x, bbox_local[0].maxPt.x), genRand(bbox_local[0].minPt.y, bbox_local[0].maxPt.y));

		// if the sampled point is outside the valid region, discard it.
		if (!withinPolygon(linkage_region_pts_local[0], points[3])) return false;

		// sample P4 within the valid region as the local coordinate of a circle point
		points[4] = glm::dvec2(genRand(bbox_local[0].minPt.x, bbox_local[0].maxPt.x), genRand(bbox_local[0].minPt.y, bbox_local[0].maxPt.y));

		// if the sampled point is outside the valid region, discard it.
		if (!withinPolygon(linkage_region_pts_local[0], points[4])) return false;

		// sample P5 within the valid region as the local coordinate of a circle point
		points[5] = glm::dvec2(genRand(bbox_local[1].minPt.x, bbox_local[1].maxPt.x), genRand(bbox_local[1].minPt.y, bbox_local[1].maxPt.y));

		// if the sampled point is outside the valid region, discard it.
		if (!withinPolygon(linkage_region_pts_local[1], points[5])) return false;

		// sample P6 within the valid region as the local coordinate of a circle point
		points[6] = glm::dvec2(genRand(bbox_local[1].minPt.x, bbox_local[1].maxPt.x), genRand(bbox_local[1].minPt.y, bbox_local[1].maxPt.y));

		// if the sampled point is outside the valid region, discard it.
		if (!withinPolygon(linkage_region_pts_local[1], points[6])) return false;







		//////////////////////////////////////////////////////////////////////////
		// DEBUG
		/*
		points[0] = glm::dvec2(38.55, 7.85);
		points[1] = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(34.85, 12.05, 1));
		points[3] = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(43.05, 12.4, 1));
		points[4] = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(51.65, 15.45, 1));
		points[5] = glm::dvec2(glm::inverse(poses[1][0]) * glm::dvec3(43, 19, 1));
		points[6] = glm::dvec2(glm::inverse(poses[1][0]) * glm::dvec3(49.75, 24.05, 1));

		// calculate transformation matrix for link P1-P3-P5
		glm::dmat3x3 T = calculateTransMatrix(glm::dvec2(43.05, 12.4), glm::dvec2(43, 19));
		glm::dmat3x3 inv_T = glm::inverse(T);
		points[2] = glm::dvec2(inv_T * glm::dvec3(44.05, 4.35, 1));


		std::vector<glm::dvec2> region_local_T(linkage_region_pts.size());
		for (int j = 0; j < linkage_region_pts.size(); j++) {
			region_local_T[j] = glm::dvec2(inv_T * glm::dvec3(linkage_region_pts[j], 1));
		}
		BBox bbox_local_T = boundingBox(region_local_T);
		*/
		//////////////////////////////////////////////////////////////////////////

		glm::dvec2 P3(poses[0][0] * glm::vec3(points[3], 1));
		glm::dvec2 P5(poses[1][0] * glm::vec3(points[5], 1));
		glm::dmat3x3 T = calculateTransMatrix(P3, P5);
		glm::dmat3x3 inv_T = glm::inverse(T);
		std::vector<glm::dvec2> region_local_T(linkage_region_pts.size());
		for (int j = 0; j < linkage_region_pts.size(); j++) {
			region_local_T[j] = glm::dvec2(inv_T * glm::dvec3(linkage_region_pts[j], 1));
		}
		BBox bbox_local_T = boundingBox(region_local_T);

		// sample P2 within the valid region as the local coordinate of a circle point
		points[2] = glm::dvec2(genRand(bbox_local_T.minPt.x, bbox_local_T.maxPt.x), genRand(bbox_local_T.minPt.y, bbox_local_T.maxPt.y));

		// if the sampled point is outside the valid region, discard it.
		if (!withinPolygon(region_local_T, points[2])) return false;



		// setup the initial parameters for optimization
		column_vector starting_point(points.size() * 2);
		column_vector lower_bound(points.size() * 2);
		column_vector upper_bound(points.size() * 2);
		for (int i = 0; i < points.size(); i++) {
			starting_point(i * 2, 0) = points[i].x;
			starting_point(i * 2 + 1, 0) = points[i].y;
		}
		lower_bound(0, 0) = bbox_world.minPt.x;
		lower_bound(1, 0) = bbox_world.minPt.y;
		lower_bound(2, 0) = bbox_local[0].minPt.x;
		lower_bound(3, 0) = bbox_local[0].minPt.y;
		lower_bound(4, 0) = bbox_local_T.minPt.x;
		lower_bound(5, 0) = bbox_local_T.minPt.y;
		lower_bound(6, 0) = bbox_local[0].minPt.x;
		lower_bound(7, 0) = bbox_local[0].minPt.y;
		lower_bound(8, 0) = bbox_local[0].minPt.x;
		lower_bound(9, 0) = bbox_local[0].minPt.y;
		lower_bound(10, 0) = bbox_local[1].minPt.x;
		lower_bound(11, 0) = bbox_local[1].minPt.y;
		lower_bound(12, 0) = bbox_local[1].minPt.x;
		lower_bound(13, 0) = bbox_local[1].minPt.y;
		
		upper_bound(0, 0) = bbox_world.maxPt.x;
		upper_bound(1, 0) = bbox_world.maxPt.y;
		upper_bound(2, 0) = bbox_local[0].maxPt.x;
		upper_bound(3, 0) = bbox_local[0].maxPt.y;
		upper_bound(4, 0) = bbox_local_T.maxPt.x;
		upper_bound(5, 0) = bbox_local_T.maxPt.y;
		upper_bound(6, 0) = bbox_local[0].maxPt.x;
		upper_bound(7, 0) = bbox_local[0].maxPt.y;
		upper_bound(8, 0) = bbox_local[0].maxPt.x;
		upper_bound(9, 0) = bbox_local[0].maxPt.y;
		upper_bound(10, 0) = bbox_local[1].maxPt.x;
		upper_bound(11, 0) = bbox_local[1].maxPt.y;
		upper_bound(12, 0) = bbox_local[1].maxPt.x;
		upper_bound(13, 0) = bbox_local[1].maxPt.y;
		
		double min_range = std::numeric_limits<double>::max();
		for (int i = 0; i < points.size() * 2; i++) {
			min_range = std::min(min_range, upper_bound(i, 0) - lower_bound(i, 0));
		}

		try {
			find_min_bobyqa(SolverForWattI(poses), starting_point, 32, lower_bound, upper_bound, min_range * 0.19, min_range * 0.0001, 1000);
			
			//check(poses, starting_point);

			points[0] = glm::dvec2(starting_point(0, 0), starting_point(1, 0));
			points[1] = glm::dvec2(poses[0][0] * glm::dvec3(starting_point(2, 0), starting_point(3, 0), 1));
			points[3] = glm::dvec2(poses[0][0] * glm::dvec3(starting_point(6, 0), starting_point(7, 0), 1));
			points[4] = glm::dvec2(poses[0][0] * glm::dvec3(starting_point(8, 0), starting_point(9, 0), 1));
			points[5] = glm::dvec2(poses[1][0] * glm::dvec3(starting_point(10, 0), starting_point(11, 0), 1));
			points[6] = glm::dvec2(poses[1][0] * glm::dvec3(starting_point(12, 0), starting_point(13, 0), 1));

			glm::dmat3x3 T = calculateTransMatrix(points[3], points[5]);
			points[2] = glm::dvec2(T * glm::dvec3(starting_point(4, 0), starting_point(5, 0), 1));
		}
		catch (std::exception& e) {
			return false;
		}

		return true;
	}

	Solution LinkageSynthesisWattI::findBestSolution(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<Solution>& solutions, std::vector<Object25D>& fixed_body_pts, const std::vector<Object25D>& body_pts, double position_error_weight, double orientation_error_weight, double linkage_location_weight, double smoothness_weight, double size_weight) {
		// select the best solution based on the trajectory
		if (solutions.size() > 0) {
			return solutions[0];
			/*
			double min_cost = std::numeric_limits<double>::max();
			int best = -1;
			for (int i = 0; i < solutions.size(); i++) {
				double position_error = solutions[i].position_error;
				double orientation_error = solutions[i].orientation_error;
				double linkage_location = solutions[i].dist;
				double tortuosity = tortuosityOfTrajectory(solutions[i].poses, solutions[i].fixed_point[0], solutions[i].fixed_point[1], solutions[i].moving_point[0], solutions[i].moving_point[1], body_pts);
				double size = glm::length(solutions[i].fixed_point[0] - solutions[i].moving_point[0]) + glm::length(solutions[i].fixed_point[1] - solutions[i].moving_point[1]) + glm::length(solutions[i].moving_point[0] - solutions[i].moving_point[1]);
				double cost = position_error * position_error_weight + orientation_error * orientation_error_weight + linkage_location * linkage_location_weight + tortuosity * smoothness_weight + size * size_weight;
				if (cost < min_cost) {
					min_cost = cost;
					best = i;
				}
			}

			return solutions[best];
			*/
		}
		else {
			return Solution({ { 0, 0 }, { 2, 0 }, { 0, 2 }, { 2, 2 }, { 1, 3 }, { 3, 3 }, { 3, 5 } }, 0, 0, 0, poses);
		}
	}

	int LinkageSynthesisWattI::getType(const std::vector<glm::dvec2>& points) {
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

	/**
	* Check if the linkage has a rotatable crank defect.
	* If the crank is not fully rotatable, true is returned.
	*/
	bool LinkageSynthesisWattI::checkRotatableCrankDefect(const std::vector<glm::dvec2>& points) {
		int linkage_type = getType(points);
		int linkage_type2 = getType({ points[3], points[4], points[5], points[6] });

		if ((linkage_type == 0 || linkage_type == 1) && (linkage_type2 == 0 || linkage_type2 == 1)) {
			return false;
		}
		else {
			return true;
		}
	}

	std::pair<double, double> LinkageSynthesisWattI::checkRange(const std::vector<glm::dvec2>& points) {
		double g = glm::length(points[0] - points[1]);
		double a = glm::length(points[0] - points[2]);
		double b = glm::length(points[1] - points[3]);
		double h = glm::length(points[2] - points[3]);

		double T1 = g + h - a - b;
		double T2 = b + g - a - h;
		double T3 = b + h - a - g;

		double theta_min = 0;
		double theta_max = M_PI * 2;

		int linkage_type = getType(points);
		if (linkage_type == 2) {
			if (crossProduct(points[0] - points[2], points[1] - points[0]) >= 0) {
				theta_min = acos((a * a + g * g - (h - b) * (h - b)) / 2 / a / g);
				theta_max = acos((a * a + g * g - (h + b) * (h + b)) / 2 / a / g);
			}
			else {
				theta_min = -acos((a * a + g * g - (h + b) * (h + b)) / 2 / a / g);
				theta_max = -acos((a * a + g * g - (h - b) * (h - b)) / 2 / a / g);
			}
		}
		else if (linkage_type == 3) {
			if (crossProduct(points[0] - points[2], points[1] - points[0]) >= 0) {
				theta_min = acos((a * a + g * g - (b - h) * (b - h)) / 2 / a / g);
				theta_max = acos((a * a + g * g - (b + h) * (b + h)) / 2 / a / g);
			}
			else {
				theta_min = -acos((a * a + g * g - (b + h) * (b + h)) / 2 / a / g);
				theta_max = -acos((a * a + g * g - (b - h) * (b - h)) / 2 / a / g);
			}
		}
		else if (linkage_type == 4 || linkage_type == 7) {
			theta_max = acos((a * a + g * g - (b + h) * (b + h)) / 2 / a / g);
			theta_min = -theta_max;
		}
		else if (linkage_type == 5) {
			theta_min = acos((a * a + g * g - (b - h) * (b - h)) / 2 / a / g);
			theta_max = M_PI * 2 - theta_min;
		}
		else if (linkage_type == 6) {
			theta_min = acos((a * a + g * g - (h - b) * (h - b)) / 2 / a / g);
			theta_max = M_PI * 2 - theta_min;
		}

		return{ theta_min, theta_max };
	}

	bool LinkageSynthesisWattI::checkOrderDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points) {
		glm::dvec2 inv_W = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(points[2], 1));

		int linkage_type = getType(points);
		std::pair<double, double> range = checkRange(points);

		double total_cw = 0;
		double total_ccw = 0;
		double prev = 0;
		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving crank in the world coordinate system
			glm::dvec2 X = glm::dvec2(poses[0][i] * glm::dvec3(inv_W, 1));
			//std::cout << X.x << "," << X.y << std::endl;

			// calculate the direction from the ground pivot (center point) of the driving crank to the circle point
			glm::dvec2 dir = X - points[0];

			// calculate its angle
			double theta = atan2(dir.y, dir.x);

			if (theta >= prev) {
				if (linkage_type == 0 || linkage_type == 2) {
					total_cw += M_PI * 2 - theta + prev;
					total_ccw += theta - prev;
				}
				else if (linkage_type == 2 || linkage_type == 3 || linkage_type == 4 || linkage_type == 7) {
					total_cw = M_PI * 999; // out of range
					total_ccw += theta - prev;
				}
				else if (linkage_type == 5 || linkage_type == 6) {
					if (theta < range.first) {
						theta += M_PI * 2;
					}
					total_cw = M_PI * 999; // out of range
					total_ccw += theta - prev;
				}
			}
			else {
				if (linkage_type == 0 || linkage_type == 2) {
					total_cw += prev - theta;
					total_ccw += M_PI * 2 - prev + theta;
				}
				else if (linkage_type == 2 || linkage_type == 3 || linkage_type == 4 || linkage_type == 7) {
					total_cw += prev - theta;
					total_ccw = M_PI * 999;	// out of range
				}
				else if (linkage_type == 5 || linkage_type == 6) {
					if (theta < range.first) {
						theta += M_PI * 2;
					}
					total_cw += prev - theta;
					total_ccw = M_PI * 999;	// out of range
				}
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
	bool LinkageSynthesisWattI::checkBranchDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points) {
		int type = getType(points);

		// drag-link and crank-rocker always do not have a branch defect
		if (type == 0 || type == 1) return false;

		int orig_sign = 1;

		// calculate the local coordinates of the circle points
		glm::dvec2 p3 = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(points[3], 1));
		glm::dvec2 p5 = glm::dvec2(glm::inverse(poses[1][0]) * glm::dvec3(points[5], 1));

		// calculate transformation matrix for link P1-P3-P5
		glm::dmat3x3 T = calculateTransMatrix(points[3], points[5]);
		glm::dvec2 p2 = glm::dvec2(glm::inverse(T) * glm::dvec3(points[2], 1));

		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 P3 = glm::dvec2(poses[0][i] * glm::dvec3(p3, 1));
			glm::dvec2 P5 = glm::dvec2(poses[1][i] * glm::dvec3(p5, 1));

			// calculate transformation matrix for link P1-P3-P5
			glm::dmat3x3 T = calculateTransMatrix(P3, P5);
			glm::dvec2 P2 = glm::dvec2(T * glm::dvec3(p2, 1));

			// calculate its sign
			if (i == 0) {
				orig_sign = crossProduct(P3 - P2, points[1] - P3) >= 0 ? 1 : -1;
			}
			else {
				int sign = crossProduct(P3 - P2, points[1] - P3) >= 0 ? 1 : -1;
				if (sign != orig_sign) {
					return true;
				}
			}
		}

		return false;
	}

	bool LinkageSynthesisWattI::checkCircuitDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points) {
		int type = getType(points);

		int orig_sign = 1;
		int orig_sign2 = 1;

		// calculate the local coordinates of the circle points
		glm::dvec2 p3 = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(points[3], 1));
		glm::dvec2 p4 = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(points[4], 1));
		glm::dvec2 p5 = glm::dvec2(glm::inverse(poses[1][0]) * glm::dvec3(points[5], 1));
		glm::dvec2 p6 = glm::dvec2(glm::inverse(poses[1][0]) * glm::dvec3(points[6], 1));

		// calculate transformation matrix for link P1-P3-P5
		glm::dmat3x3 T = calculateTransMatrix(points[3], points[5]);
		glm::dvec2 p2 = glm::dvec2(glm::inverse(T) * glm::dvec3(points[2], 1));

		for (int i = 0; i < poses[0].size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 P3 = glm::dvec2(poses[0][i] * glm::dvec3(p3, 1));
			glm::dvec2 P4 = glm::dvec2(poses[0][i] * glm::dvec3(p4, 1));
			glm::dvec2 P5 = glm::dvec2(poses[1][i] * glm::dvec3(p5, 1));
			glm::dvec2 P6 = glm::dvec2(poses[1][i] * glm::dvec3(p6, 1));

			// calculate transformation matrix for link P1-P3-P5
			glm::dmat3x3 T = calculateTransMatrix(P3, P5);
			glm::dvec2 P2 = glm::dvec2(T * glm::dvec3(p2, 1));

			// calculate its sign
			if (i == 0) {
				if (type == 0) {
					orig_sign = crossProduct(P2 - points[0], P3 - P2) >= 0 ? 1 : -1;
				}
				else if (type == 1) {
					orig_sign = crossProduct(P3 - P2, points[1] - P3) >= 0 ? 1 : -1;
				}
				else if (type == 2) {
					orig_sign = crossProduct(points[0] - points[1], P2 - points[0]) >= 0 ? 1 : -1;
				}
				else if (type == 3) {
					orig_sign = crossProduct(points[1] - P3, points[0] - points[1]) >= 0 ? 1 : -1;
				}

				orig_sign2 = crossProduct(P6 - P5, P4 - P6) >= 0 ? 1 : -1;
			}
			else {
				int sign, sign2;
				if (type == 0) {
					sign = crossProduct(P2 - points[0], P3 - P2) >= 0 ? 1 : -1;
				}
				else if (type == 1) {
					sign = crossProduct(P3 - P2, points[1] - P3) >= 0 ? 1 : -1;
				}
				else if (type == 2) {
					sign = crossProduct(points[0] - points[1], P2 - points[0]) >= 0 ? 1 : -1;
				}
				else if (type == 3) {
					sign = crossProduct(points[1] - P3, points[0] - points[1]) >= 0 ? 1 : -1;
				}
				else {
					sign = orig_sign;
				}

				sign2 = crossProduct(P6 - P5, P4 - P6) >= 0 ? 1 : -1;

				if (sign != orig_sign || sign2 != orig_sign2) return true;
			}
		}

		return false;
	}

	/**
	* Construct a linkage.
	*/
	Kinematics LinkageSynthesisWattI::constructKinematics(const std::vector<glm::dvec2>& points, const std::vector<std::vector<int>>& zorder, const std::vector<Object25D>& body_pts, bool connect_joints, std::vector<Object25D>& fixed_body_pts) {
		Kinematics kin;
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(0, true, points[0], 1)));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(1, true, points[1], 1)));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(2, false, points[2], 1)));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(3, false, points[3], 1)));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(4, false, points[4], 1)));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(5, false, points[5], 1)));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(6, false, points[6], 1)));
		kin.diagram.addLink(true, { kin.diagram.joints[0], kin.diagram.joints[2] }, 1);
		kin.diagram.addLink(false, { kin.diagram.joints[1], kin.diagram.joints[3], kin.diagram.joints[4] }, 1);
		kin.diagram.addLink(false, { kin.diagram.joints[2], kin.diagram.joints[3], kin.diagram.joints[5] }, 1);
		kin.diagram.addLink(false, kin.diagram.joints[4], kin.diagram.joints[6], 1);
		kin.diagram.addLink(false, kin.diagram.joints[5], kin.diagram.joints[6], 1);

		// update the geometry
		updateBodies(kin, body_pts);
		if (connect_joints) {
			kin.diagram.connectJointsToBodies(fixed_body_pts, zorder);
		}

		// add the fixed rigid bodies
		for (int i = 0; i < fixed_body_pts.size(); i++) {
			kin.diagram.addBody(kin.diagram.joints[0], kin.diagram.joints[1], fixed_body_pts[i]);
		}

		return kin;
	}

	/**
	* update bodies.
	*/
	void LinkageSynthesisWattI::updateBodies(Kinematics& kin, const std::vector<Object25D>& body_pts) {
		kin.diagram.bodies.clear();
		kin.diagram.addBody(kin.diagram.joints[1], kin.diagram.joints[3], body_pts[0]);
		kin.diagram.addBody(kin.diagram.joints[5], kin.diagram.joints[6], body_pts[1]);
	}

	bool LinkageSynthesisWattI::checkCollision(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points, std::vector<Object25D> fixed_body_pts, const std::vector<Object25D>& body_pts, int collision_check_type) {
		kinematics::Kinematics kinematics = constructKinematics(points, {}, body_pts, (collision_check_type == 1 || collision_check_type == 3), fixed_body_pts);

		// add the fixed rigid bodies to the fixed joints of all the linkages
		for (int i = 0; i < fixed_body_pts.size(); i++) {
			kinematics.diagram.addBody(kinematics.diagram.joints[0], kinematics.diagram.joints[1], fixed_body_pts[i]);
		}

		kinematics.diagram.initialize();

		// calculate the rotational angle of the driving crank for 1st, 2nd, and last poses
		// i.e., angles[0] = first pose, angles[1] = second pose, angles[2] = last pose
		std::vector<double> angles(3);

		glm::dvec2 p3(glm::inverse(poses[0][0]) * glm::dvec3(points[3], 1));
		glm::dvec2 p5(glm::inverse(poses[1][0]) * glm::dvec3(points[5], 1));
		glm::dmat3x3 T = calculateTransMatrix(points[3], points[5]);
		glm::dvec2 p2(glm::inverse(T) * glm::dvec3(points[2], 1));

		for (int i = 0; i < 2; i++) {
			glm::dvec2 P3(poses[0][i] * glm::dvec3(p3, 1));
			glm::dvec2 P5(poses[1][i] * glm::dvec3(p5, 1));
			glm::dmat3x3 T = calculateTransMatrix(P3, P5);
			glm::dvec2 P2(T * glm::dvec3(p2, 1));
			
			angles[i] = atan2(P2.y - points[0].y, P2.x - points[0].x);
		}
		{
			glm::dvec2 P3(poses[0].back() * glm::dvec3(p3, 1));
			glm::dvec2 P5(poses[1].back() * glm::dvec3(p5, 1));
			glm::dmat3x3 T = calculateTransMatrix(P3, P5);
			glm::dvec2 P2(T * glm::dvec3(p2, 1));

			angles[2] = atan2(P2.y - points[0].y, P2.x - points[0].x);
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
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] >= 0 && (poses[0].size() >= 3 && angles[1] >= angles[2] || poses[0].size() == 2 && angles[1] - angles[0] > M_PI)) {
			type = 4;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] < 0 && (poses[0].size() >= 3 && angles[1] < angles[2] || poses[0].size() == 2 && angles[0] - angles[1] > M_PI)) {
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
				kinematics.stepForward(true, false);
			}
			catch (char* ex) {
				// if only some of the poses are reached before collision, the collision is detected.
				kinematics.clear();
				return true;
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

	double LinkageSynthesisWattI::check(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points) {
		glm::dvec2 P0 = points[0];
		glm::dvec2 P1 = points[1];
		glm::dvec2 P2 = points[2];
		glm::dvec2 P3 = points[3];
		glm::dvec2 P4 = points[4];
		glm::dvec2 P5 = points[5];
		glm::dvec2 P6 = points[6];

		glm::dvec2 p1(glm::inverse(poses[0][0]) * glm::dvec3(P1, 1));
		glm::dvec2 p3(glm::inverse(poses[0][0]) * glm::dvec3(P3, 1));
		glm::dvec2 p4(glm::inverse(poses[0][0]) * glm::dvec3(P4, 1));
		glm::dvec2 p5(glm::inverse(poses[1][0]) * glm::dvec3(P5, 1));
		glm::dvec2 p6(glm::inverse(poses[1][0]) * glm::dvec3(P6, 1));

		// calculate transformation matrix for link P1-P3-P5
		glm::dmat3x3 T = calculateTransMatrix(P3, P5);
		glm::dvec2 p2(glm::inverse(T) * glm::dvec3(P2, 1));
		
		std::vector<double> lengths;
		lengths.push_back(P1.x * P1.x);
		lengths.push_back(P1.y * P1.y);
		lengths.push_back(glm::length2(P2 - P0));
		lengths.push_back(glm::length2(P5 - P3));
		lengths.push_back(glm::length2(P6 - P4));

		lengths.push_back(glm::length2(P3 - P1));
		lengths.push_back(glm::length2(P3 - P2));
		lengths.push_back(glm::length2(P4 - P3));
		lengths.push_back(glm::length2(P5 - P2));
		lengths.push_back(glm::length2(P6 - P5));
		lengths.push_back(glm::length2(P1 - P0));

		double ans = 0.0;
		for (int i = 1; i < poses.size(); i++) {
			glm::dvec2 P1b(poses[0][i] * glm::dvec3(p1, 1));
			glm::dvec2 P3b(poses[0][i] * glm::dvec3(p3, 1));
			glm::dvec2 P4b(poses[0][i] * glm::dvec3(p4, 1));
			glm::dvec2 P5b(poses[1][i] * glm::dvec3(p5, 1));
			glm::dvec2 P6b(poses[1][i] * glm::dvec3(p6, 1));

			// calculate transformation matrix for link P1-P3-P5
			glm::dmat3x3 Tb = calculateTransMatrix(P3b, P5b);
			glm::dvec2 P2b(Tb * glm::dvec3(p2, 1));

			std::vector<double> lengths2;
			lengths2.push_back(P1b.x * P1b.x);
			lengths2.push_back(P1b.y * P1b.y);
			lengths2.push_back(glm::length2(P2b - P0));
			lengths2.push_back(glm::length2(P5b - P3b));
			lengths2.push_back(glm::length2(P6b - P4b));

			lengths2.push_back(glm::length2(P3b - P1b));
			lengths2.push_back(glm::length2(P3b - P2b));
			lengths2.push_back(glm::length2(P4b - P3b));
			lengths2.push_back(glm::length2(P5b - P2b));
			lengths2.push_back(glm::length2(P6b - P5b));
			lengths2.push_back(glm::length2(P1b - P0));

			//for (int i = 0; i < lengths.size(); i++) {
			for (int i = 0; i < 5; i++) {
				ans += (lengths[i] - lengths2[i]) * (lengths[i] - lengths2[i]);
			}
		}

		return ans;
	}

}
