#include "LinkageSynthesis4R.h"
#include "KinematicUtils.h"
#include "Kinematics.h"
#include "PinJoint.h"
#include "SliderHinge.h"
#include "BoundingBox.h"
#include "LeastSquareSolver.h"
#include <opencv2/opencv.hpp>

namespace kinematics {

	/**
	* Calculate solutions of 4R linkage given three poses.
	*
	* @param poses			three poses
	* @param solutions1	the output solutions for the world coordinates of the driving crank at the first pose, each of which contains a pair of the center point and the circle point
	* @param solutions2	the output solutions for the world coordinates of the follower at the first pose, each of which contains a pair of the center point and the circle point
	*/
	void LinkageSynthesis4R::calculateSolution(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, int num_samples, const std::vector<std::vector<glm::dvec2>>& fixed_body_pts, const std::vector<glm::dvec2>& body_pts, std::vector<std::pair<double, double>>& sigmas, bool rotatable_crank, bool avoid_branch_defect, double min_link_length, std::vector<Solution>& solutions) {
		solutions.clear();

		srand(0);

		// calculate the center of the valid regions
		BBox bbox_world = boundingBox(linkage_region_pts);
		glm::dvec2 bbox_world_center = bbox_world.center();

		// convert the coordinates of the regions to the local coordinate system of the first pose
		glm::dmat3x3 inv_pose0 = glm::inverse(poses[0]);
		std::vector<glm::dvec2> region_local(linkage_region_pts.size());
		for (int i = 0; i < linkage_region_pts.size(); i++) {
			region_local[i] = glm::dvec2(inv_pose0 * glm::dvec3(linkage_region_pts[i], 1));
		}

		// calculate the bounding boxes of the regions
		BBox bbox_local = boundingBox(region_local);

		int cnt = 0;
		for (int scale = 1; scale <= 5 && cnt < num_samples; scale++) {
			// calculate the enlarged linkage region for the sampling region
			std::vector<glm::dvec2> enlarged_linkage_region_pts;
			for (int i = 0; i < linkage_region_pts.size(); i++) {
				enlarged_linkage_region_pts.push_back((linkage_region_pts[i] - bbox_world_center) * (double)scale + bbox_world_center);
			}

			// calculate the bounding boxe of the valid regions
			BBox enlarged_bbox_world = boundingBox(enlarged_linkage_region_pts);

			// calculate the distace transform of the linkage region
			cv::Mat img(enlarged_bbox_world.height() + 1, enlarged_bbox_world.width() + 1, CV_8U, cv::Scalar(255));
			std::vector<std::vector<cv::Point>> pts(1);
			for (int i = 0; i < linkage_region_pts.size(); i++) {
				double x = linkage_region_pts[i].x - enlarged_bbox_world.minPt.x;
				double y = linkage_region_pts[i].y - enlarged_bbox_world.minPt.y;
				pts[0].push_back(cv::Point(x, y));
			}
			cv::fillPoly(img, pts, cv::Scalar(0), 4);
			cv::Mat distMap;
			cv::distanceTransform(img, distMap, CV_DIST_L2, 3);
			//cv::imwrite("test2.png", img);
			//cv::imwrite("test.png", distMap);
			distMap.convertTo(distMap, CV_64F);

			// convert the coordinates of the enlarged regions to the local coordinate system of the first pose
			std::vector<glm::dvec2> enlarged_region_local(enlarged_linkage_region_pts.size());
			for (int i = 0; i < enlarged_linkage_region_pts.size(); i++) {
				enlarged_region_local[i] = glm::dvec2(inv_pose0 * glm::dvec3(enlarged_linkage_region_pts[i], 1));
			}

			// calculate the bounding boxes of the enlarged regions
			BBox enlarged_bbox_local = boundingBox(enlarged_region_local);
			
			for (int iter = 0; iter < num_samples * 100 && cnt < num_samples; iter++) {
				printf("\rsampling %d/%d", cnt, (scale - 1) * num_samples * 100 + iter + 1);

				// perturbe the poses a little
				// HACK: 本来なら、bodyの座標を関数に渡し、関数側でpertubeしてからposeを計算すべきか？
				//       とりあえず、回転はperturbしていない。
				std::vector<glm::dmat3x3> perturbed_poses = poses;
				double position_error = 0.0;
				double orientation_error = 0.0;
				for (int i = 0; i < poses.size(); i++) {
					double e1 = 0;
					double e2 = 0;
					double delta_theta = 0;
					if (i == 0) {
						e1 = genNormal(0, sigmas[0].first);
						e2 = genNormal(0, sigmas[0].first);
						delta_theta = genNormal(0, sigmas[0].second);
					}
					else if (i == poses.size() - 1) {
						e1 = genNormal(0, sigmas[2].first);
						e2 = genNormal(0, sigmas[2].first);
						delta_theta = genNormal(0, sigmas[2].second);
					}
					else {
						e1 = genNormal(0, sigmas[1].first);
						e2 = genNormal(0, sigmas[1].first);
						delta_theta = genNormal(0, sigmas[1].second);
					}

					perturbed_poses[i][2][0] += e1;
					perturbed_poses[i][2][1] += e2;
					position_error += std::sqrt(e1 * e1 + e2 * e2);

					double theta = atan2(poses[i][0][1], poses[i][0][0]) + delta_theta;
					perturbed_poses[i][0][0] = cos(theta);
					perturbed_poses[i][0][1] = sin(theta);
					perturbed_poses[i][1][0] = -sin(theta);
					perturbed_poses[i][1][1] = cos(theta);
					orientation_error += abs(delta_theta);
				}

				// sample a linkage
				glm::dvec2 A0, A1;
				if (poses.size() == 2) {
					if (!sampleLinkForTwoPoses(perturbed_poses, enlarged_linkage_region_pts, enlarged_region_local, enlarged_bbox_world, enlarged_bbox_local, A0, A1)) continue;
				}
				else if (poses.size() == 3) {
					if (!sampleLinkForThreePoses(perturbed_poses, enlarged_linkage_region_pts, enlarged_region_local, enlarged_bbox_local, A0, A1)) continue;
				}
				else {
					if (!sampleLink(perturbed_poses, enlarged_linkage_region_pts, enlarged_region_local, enlarged_bbox_world, enlarged_bbox_local, A0, A1)) continue;
				}

				glm::dvec2 B0, B1;
				if (poses.size() == 2) {
					if (!sampleLinkForTwoPoses(perturbed_poses, enlarged_linkage_region_pts, enlarged_region_local, enlarged_bbox_world, enlarged_bbox_local, B0, B1)) continue;
				}
				else if (poses.size() == 3) {
					if (!sampleLinkForThreePoses(perturbed_poses, enlarged_linkage_region_pts, enlarged_region_local, enlarged_bbox_local, B0, B1)) continue;
				}
				else {
					if (!sampleLink(perturbed_poses, enlarged_linkage_region_pts, enlarged_region_local, enlarged_bbox_world, enlarged_bbox_local, B0, B1)) continue;
				}

				// check hard constraints
				if (glm::length(A0 - B0) < min_link_length) continue;
				if (glm::length(A1 - B1) < min_link_length) continue;

				if (checkFolding(A0, B0, A1, B1)) continue;
				if (rotatable_crank && checkRotatableCrankDefect(A0, B0, A1, B1)) continue;
				if (avoid_branch_defect && checkBranchDefect(perturbed_poses, A0, B0, A1, B1)) continue;
				if (checkCircuitDefect(perturbed_poses, A0, B0, A1, B1)) continue;
				if (checkOrderDefect(perturbed_poses, A0, B0, A1, B1)) continue;

				// collision check
				if (checkCollision(perturbed_poses, A0, B0, A1, B1, fixed_body_pts, body_pts)) continue;

				// calculate the distance of the joints from the user-specified linkage region
				double dist = 0.0;
				dist += distMap.at<double>(A0.y - enlarged_bbox_world.minPt.y, A0.x - enlarged_bbox_world.minPt.x);
				dist += distMap.at<double>(A1.y - enlarged_bbox_world.minPt.y, A1.x - enlarged_bbox_world.minPt.x);
				dist += distMap.at<double>(B0.y - enlarged_bbox_world.minPt.y, B0.x - enlarged_bbox_world.minPt.x);
				dist += distMap.at<double>(B1.y - enlarged_bbox_world.minPt.y, B1.x - enlarged_bbox_world.minPt.x);

				solutions.push_back(Solution(A0, A1, B0, B1, position_error, orientation_error, dist, perturbed_poses));
				cnt++;
			}
		}
		printf("\n");
	}

	bool LinkageSynthesis4R::sampleLink(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_region_pts_local, const BBox& bbox_world, const BBox& bbox_local, glm::dvec2& A0, glm::dvec2& A1) {
		// sample a point within the valid region as the world coordinates of a center point
		A0 = glm::dvec2(genRand(bbox_world.minPt.x, bbox_world.maxPt.x), genRand(bbox_world.minPt.y, bbox_world.maxPt.y));

		// if the sampled point is outside the valid region, discard it.
		if (!withinPolygon(linkage_region_pts, A0)) return false;

		// sample a point within the valid region as the local coordinate of a circle point
		glm::dvec2 a(genRand(bbox_local.minPt.x, bbox_local.maxPt.x), genRand(bbox_local.minPt.y, bbox_local.maxPt.y));

		// if the sampled point is outside the valid region, discard it.
		if (!withinPolygon(linkage_region_pts_local, a)) return false;

		// setup the initial parameters for optimization
		column_vector starting_point(4);
		column_vector lower_bound(4);
		column_vector upper_bound(4);
		starting_point(0, 0) = A0.x;
		starting_point(1, 0) = A0.y;
		starting_point(2, 0) = a.x;
		starting_point(3, 0) = a.y;
		lower_bound(0, 0) = bbox_world.minPt.x;
		lower_bound(1, 0) = bbox_world.minPt.y;
		lower_bound(2, 0) = bbox_local.minPt.x;
		lower_bound(3, 0) = bbox_local.minPt.y;
		upper_bound(0, 0) = bbox_world.maxPt.x;
		upper_bound(1, 0) = bbox_world.maxPt.y;
		upper_bound(2, 0) = bbox_local.maxPt.x;
		upper_bound(3, 0) = bbox_local.maxPt.y;

		double min_range = std::numeric_limits<double>::max();
		for (int i = 0; i < 4; i++) {
			min_range = std::min(min_range, upper_bound(i, 0) - lower_bound(i, 0));
		}

		try {
			find_min_bobyqa(SolverForLink(poses), starting_point, 14, lower_bound, upper_bound, min_range * 0.19, min_range * 0.0001, 1000);

			A0.x = starting_point(0, 0);
			A0.y = starting_point(1, 0);
			a.x = starting_point(2, 0);
			a.y = starting_point(3, 0);

			// if the center point is outside the valid region, discard it.
			if (!withinPolygon(linkage_region_pts, A0)) return false;

			A1 = glm::dvec2(poses[0] * glm::dvec3(a, 1));

			// if the moving point is outside the valid region, discard it.
			if (!withinPolygon(linkage_region_pts, A1)) return false;
		}
		catch (std::exception& e) {
			return false;
		}

		return true;
	}

	bool LinkageSynthesis4R::sampleLinkForThreePoses(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_region_pts_local, const BBox& bbox, glm::dvec2& A0, glm::dvec2& A1) {
		// sample a point within the valid region as the local coordinate of a circle point
		glm::dvec2 a(genRand(bbox.minPt.x, bbox.maxPt.x), genRand(bbox.minPt.y, bbox.maxPt.y));

		// if the sampled point is outside the valid region, discard it.
		if (!withinPolygon(linkage_region_pts_local, a)) return false;

		A1 = glm::dvec2(poses[0] * glm::dvec3(a, 1));
		glm::dvec2 A2(poses[1] * glm::dvec3(a, 1));
		glm::dvec2 A3(poses[2] * glm::dvec3(a, 1));

		try {
			A0 = circleCenterFromThreePoints(A1, A2, A3);

			// if the center point is outside the valid region, discard it.
			if (!withinPolygon(linkage_region_pts, A0)) return false;

			// if the moving point is outside the valid region, discard it.
			if (!withinPolygon(linkage_region_pts, A1)) return false;
		}
		catch (char* ex) {
			return false;
		}

		return true;
	}

	bool LinkageSynthesis4R::sampleLinkForTwoPoses(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_region_pts_local, const BBox& bbox_world, const BBox& bbox_local, glm::dvec2& A0, glm::dvec2& A1) {
		// sample a point within the valid region as the local coordinate of a circle point
		glm::dvec2 a(genRand(bbox_local.minPt.x, bbox_local.maxPt.x), genRand(bbox_local.minPt.y, bbox_local.maxPt.y));

		// if the sampled point is outside the valid region, discard it.
		if (!withinPolygon(linkage_region_pts_local, a)) return false;

		A1 = glm::dvec2(poses[0] * glm::dvec3(a, 1));
		glm::dvec2 A2(poses[1] * glm::dvec3(a, 1));

		// sample a poing within the region as the fixed point
		A0 = glm::dvec2(genRand(bbox_world.minPt.x, bbox_world.maxPt.x), genRand(bbox_world.minPt.y, bbox_world.maxPt.y));

		// if the sampled point is outside the valid region, discard it.
		if (!withinPolygon(linkage_region_pts, A0)) return false;

		glm::dvec2 M = (A1 + A2) * 0.5;
		glm::dvec2 v = A1 - A2;
		v /= glm::length(v);
		glm::dvec2 h(-v.y, v.x);

		A0 = M + h * glm::dot(A0 - M, h);

		// if the center point is outside the valid region, discard it.
		if (!withinPolygon(linkage_region_pts, A0)) return false;

		return true;
	}

	Solution LinkageSynthesis4R::findBestSolution(const std::vector<glm::dmat3x3>& poses, const std::vector<Solution>& solutions, const std::vector<std::vector<glm::dvec2>>& fixed_body_pts, const std::vector<glm::dvec2>& body_pts, double position_error_weight, double orientation_error_weight, double linkage_location_weight, double smoothness_weight, double size_weight) {
		// select the best solution based on the trajectory
		if (solutions.size() > 0) {
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
		}
		else {
			return Solution({ 0, 0 }, { 0, 2 }, { 2, 0 }, { 2, 2 }, 0, 0, 0, poses);
		}
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
	int LinkageSynthesis4R::getType(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3) {
		double g = glm::length(p0 - p1);
		double a = glm::length(p0 - p2);
		double b = glm::length(p1 - p3);
		double h = glm::length(p2 - p3);

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

	bool LinkageSynthesis4R::checkFolding(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3) {
		double g = glm::length(p0 - p1);
		double a = glm::length(p0 - p2);
		double b = glm::length(p1 - p3);
		double h = glm::length(p2 - p3);

		double T1 = g + h - a - b;
		double T2 = b + g - a - h;
		double T3 = b + h - a - g;

		if (abs(T1) < 0.05 || abs(T2) < 0.05 || abs(T3) < 0.05) return true;
		else return false;
	}

	std::pair<double, double> LinkageSynthesis4R::checkRange(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3) {
		double g = glm::length(p0 - p1);
		double a = glm::length(p0 - p2);
		double b = glm::length(p1 - p3);
		double h = glm::length(p2 - p3);

		double T1 = g + h - a - b;
		double T2 = b + g - a - h;
		double T3 = b + h - a - g;

		double theta_min = 0;
		double theta_max = M_PI * 2;

		int linkage_type = getType(p0, p1, p2, p3);
		if (linkage_type == 2) {
			if (crossProduct(p0 - p2, p1 - p0) >= 0) {
				theta_min = acos((a * a + g * g - (h - b) * (h - b)) / 2 / a / g);
				theta_max = acos((a * a + g * g - (h + b) * (h + b)) / 2 / a / g);
			}
			else {
				theta_min = -acos((a * a + g * g - (h + b) * (h + b)) / 2 / a / g);
				theta_max = -acos((a * a + g * g - (h - b) * (h - b)) / 2 / a / g);
			}
		}
		else if (linkage_type == 3) {
			if (crossProduct(p0 - p2, p1 - p0) >= 0) {
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

		return { theta_min, theta_max };
	}

	/**
	* Check if the linkage has a rotatable crank defect.
	* If the crank is not fully rotatable, true is returned.
	*/
	bool LinkageSynthesis4R::checkRotatableCrankDefect(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3) {
		int linkage_type = getType(p0, p1, p2, p3);

		if (linkage_type == 0 || linkage_type == 1) {
			return false;
		}
		else {
			return true;
		}
	}

	/**
	* Check if the linkage has order defect.
	* If there is an order defect, true is returned.
	* Otherwise, false is returned.
	*/
	bool LinkageSynthesis4R::checkOrderDefect(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, bool debug) {
		glm::dvec2 inv_W = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(p2, 1));

		int linkage_type = getType(p0, p1, p2, p3);
		std::pair<double, double> range = checkRange(p0, p1, p2, p3);

		double total_cw = 0;
		double total_ccw = 0;
		double prev = 0;
		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving crank in the world coordinate system
			glm::dvec2 X = glm::dvec2(poses[i] * glm::dvec3(inv_W, 1));
			//std::cout << X.x << "," << X.y << std::endl;

			// calculate the direction from the ground pivot (center point) of the driving crank to the circle point
			glm::dvec2 dir = X - p0;

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
	bool LinkageSynthesis4R::checkBranchDefect(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, bool debug) {
		int type = getType(p0, p1, p2, p3);

		// drag-link and crank-rocker always do not have a branch defect
		if (type == 0 || type == 1) return false;

		int orig_sign = 1;

		// calculate the local coordinates of the circle points
		glm::dvec2 W1 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(p2, 1));
		glm::dvec2 W2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(p3, 1));

		if (debug) {
			std::cout << "Branch defect debug..." << std::endl;
			std::cout << "P0: (" << p0.x << ", " << p0.y << "), P1: (" << p1.x << ", " << p1.y << ")" << std::endl;
		}
		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 X1 = glm::dvec2(poses[i] * glm::dvec3(W1, 1));
			glm::dvec2 X2 = glm::dvec2(poses[i] * glm::dvec3(W2, 1));

			if (debug) {
				std::cout << "X1: (" << X1.x << "," << X1.y << "), X2: (" << X2.x << ", " << X2.y << ")" << std::endl;
				std::cout << "Sign: " << (crossProduct(X2 - p1, X1 - X2) >= 0 ? 1 : -1) << std::endl;
			}

			// calculate its sign
			if (i == 0) {
				orig_sign = crossProduct(X2 - p1, X1 - X2) >= 0 ? 1 : -1;
			}
			else {
				int sign = crossProduct(X2 - p1, X1 - X2) >= 0 ? 1 : -1;
				if (sign != orig_sign) {
					if (debug) {
						std::cout << "Sign is different" << std::endl;
					}
					return true;
				}
			}
		}

		return false;
	}

	bool LinkageSynthesis4R::checkCircuitDefect(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, bool debug) {
		int type = getType(p0, p1, p2, p3);

		// Non-grashof type does not have a circuit defect
		if (type >= 4) return false;

		int orig_sign0 = 1;
		int orig_sign1 = 1;
		int orig_sign2 = 1;
		int orig_sign3 = 1;

		// calculate the local coordinates of the circle points
		glm::dvec2 W1 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(p2, 1));
		glm::dvec2 W2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(p3, 1));

		if (debug) {
			std::cout << "Circuit defect debug..." << std::endl;
			std::cout << "P0: (" << p0.x << ", " << p0.y << "), P1: (" << p1.x << ", " << p1.y << ")" << std::endl;
		}

		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 X1 = glm::dvec2(poses[i] * glm::dvec3(W1, 1));
			glm::dvec2 X2 = glm::dvec2(poses[i] * glm::dvec3(W2, 1));

			if (debug) {
				std::cout << "X1: (" << X1.x << "," << X1.y << "), X2: (" << X2.x << ", " << X2.y << ")" << std::endl;
				std::cout << "Sign0: " << (crossProduct(p0 - p1, X1 - p0) >= 0 ? 1 : -1) << std::endl;
				std::cout << "Sign1: " << (crossProduct(p1 - X2, p0 - p1) >= 0 ? 1 : -1) << std::endl;
				std::cout << "Sign2: " << (crossProduct(X1 - p0, X2 - X1) >= 0 ? 1 : -1) << std::endl;
				std::cout << "Sign3: " << (crossProduct(X2 - X1, p1 - X2) >= 0 ? 1 : -1) << std::endl;
			}

			// calculate its sign
			if (i == 0) {
				orig_sign0 = crossProduct(p0 - p1, X1 - p0) >= 0 ? 1 : -1;
				orig_sign1 = crossProduct(p1 - X2, p0 - p1) >= 0 ? 1 : -1;
				orig_sign2 = crossProduct(X1 - p0, X2 - X1) >= 0 ? 1 : -1;
				orig_sign3 = crossProduct(X2 - X1, p1 - X2) >= 0 ? 1 : -1;
			}
			else {
				int sign0 = crossProduct(p0 - p1, X1 - p0) >= 0 ? 1 : -1;
				int sign1 = crossProduct(p1 - X2, p0 - p1) >= 0 ? 1 : -1;
				int sign2 = crossProduct(X1 - p0, X2 - X1) >= 0 ? 1 : -1;
				int sign3 = crossProduct(X2 - X1, p1 - X2) >= 0 ? 1 : -1;

				if (type == 0) {
					if (sign2 != orig_sign2 || sign3 != orig_sign3) {
						if (debug) {
							std::cout << "Sign is different" << std::endl;
						}
						return true;
					}
				}
				else if (type == 1) {
					if (sign1 != orig_sign1 || sign3 != orig_sign3) {
						if (debug) {
							std::cout << "Sign is different" << std::endl;
						}
						return true;
					}
				}
				else if (type == 2) {
					if (sign0 != orig_sign0 || sign2 != orig_sign2) {
						if (debug) {
							std::cout << "Sign is different" << std::endl;
						}
						return true;
					}
				}
				else if (type == 3) {
					if (sign0 != orig_sign0 || sign1 != orig_sign1) {
						if (debug) {
							std::cout << "Sign is different" << std::endl;
						}
						return true;
					}
				}
			}
		}

		return false;
	}

	bool LinkageSynthesis4R::checkCollision(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const std::vector<std::vector<glm::dvec2>>& fixed_body_pts, const std::vector<glm::dvec2>& body_pts) {
		kinematics::Kinematics kinematics;

		// construct a linkage
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(0, true, p0)));
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(1, true, p1)));
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(2, false, p2)));
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(3, false, p3)));
		kinematics.diagram.addLink(true, kinematics.diagram.joints[0], kinematics.diagram.joints[2]);
		kinematics.diagram.addLink(false, kinematics.diagram.joints[1], kinematics.diagram.joints[3]);
		kinematics.diagram.addLink(false, kinematics.diagram.joints[2], kinematics.diagram.joints[3]);

		// set the geometry
		for (int i = 0; i < fixed_body_pts.size(); i++) {
			kinematics.diagram.addBody(kinematics.diagram.joints[0], kinematics.diagram.joints[1], fixed_body_pts[i]);
		}
		kinematics.diagram.addBody(kinematics.diagram.joints[2], kinematics.diagram.joints[3], body_pts);

		kinematics.diagram.initialize();

		// calculate the rotational angle of the driving crank for 1st, 2nd, and last poses
		// i.e., angles[0] = first pose, angles[1] = second pose, angles[2] = last pose
		std::vector<double> angles(3);
		glm::dvec2 w(glm::inverse(poses[0]) * glm::dvec3(p2, 1));
		for (int i = 0; i < 2; i++) {
			glm::dvec2 W = glm::dvec2(poses[i] * glm::dvec3(w, 1));
			angles[i] = atan2(W.y - p0.y, W.x - p0.x);
		}
		{
			glm::dvec2 W = glm::dvec2(poses.back() * glm::dvec3(w, 1));
			angles[2] = atan2(W.y - p0.y, W.x - p0.x);
		}

		//////////////////////////////////////////
		// DEBUG
		if (false) {
			std::cout << "angles:" << std::endl;
			for (int i = 0; i < poses.size(); i++) {
				glm::dvec2 W = glm::dvec2(poses[i] * glm::dvec3(w, 1));
				double angle = atan2(W.y - p0.y, W.x - p0.x);
				std::cout << angle / M_PI * 180 << std::endl;
			}
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
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] >= 0 && angles[1] >= angles[2]) {
			type = 4;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] < 0 && angles[1] < angles[2]) {
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

		//////////////////////////////////////////
		// DEBUG
		if (false) {
			std::cout << "angles:" << std::endl;
			std::cout << "type: " << type << std::endl;
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
			double angle = atan2(kinematics.diagram.joints[2]->pos.y - p0.y, kinematics.diagram.joints[2]->pos.x - p0.x);

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

	double LinkageSynthesis4R::tortuosityOfTrajectory(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const std::vector<glm::dvec2>& body_pts) {
		// calculate the local coordinates of the body points
		glm::dmat3x3 inv_pose0 = glm::inverse(poses[0]);
		std::vector<glm::dvec2> body_pts_local(body_pts.size());
		for (int i = 0; i < body_pts.size(); i++) {
			body_pts_local[i] = glm::dvec2(inv_pose0 * glm::dvec3(body_pts[i], 1));
		}

		// calculate the length of the motion using straight lines between poses
		double length_of_straight = 0.0;
		std::vector<glm::dvec2> prev_body_pts = body_pts;
		for (int i = 1; i < poses.size(); i++) {
			std::vector<glm::dvec2> next_body_pts(body_pts.size());
			for (int k = 0; k < body_pts.size(); k++) {
				next_body_pts[k] = glm::dvec2(poses[i] * glm::dvec3(body_pts_local[k], 1));
				length_of_straight += glm::length(next_body_pts[k] - prev_body_pts[k]);
			}
			prev_body_pts = next_body_pts;
		}
		
		// create a kinematics
		kinematics::Kinematics kinematics(0.1);

		// construct a linkage
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(0, true, p0)));
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(1, true, p1)));
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(2, false, p2)));
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(3, false, p3)));
		kinematics.diagram.addLink(true, kinematics.diagram.joints[0], kinematics.diagram.joints[2]);
		kinematics.diagram.addLink(false, kinematics.diagram.joints[1], kinematics.diagram.joints[3]);
		kinematics.diagram.addLink(false, kinematics.diagram.joints[2], kinematics.diagram.joints[3]);

		// set the geometry
		kinematics.diagram.addBody(kinematics.diagram.joints[2], kinematics.diagram.joints[3], body_pts);

		kinematics.diagram.initialize();

		// initialize the trajectory of the moving body
		prev_body_pts = body_pts;
		double length_of_trajectory = 0.0;

		// calculate the rotational angle of the driving crank for 1st, 2nd, and last poses
		// i.e., angles[0] = first pose, angles[1] = second pose, angles[2] = last pose
		std::vector<double> angles(3);
		glm::dvec2 w(glm::inverse(poses[0]) * glm::dvec3(p2, 1));
		for (int i = 0; i < 2; i++) {
			glm::dvec2 W = glm::dvec2(poses[i] * glm::dvec3(w, 1));
			angles[i] = atan2(W.y - p0.y, W.x - p0.x);
		}
		{
			glm::dvec2 W = glm::dvec2(poses.back() * glm::dvec3(w, 1));
			angles[2] = atan2(W.y - p0.y, W.x - p0.x);
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
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] >= 0 && angles[1] >= angles[2]) {
			type = 4;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] < 0 && angles[1] < angles[2]) {
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
				return length_of_trajectory / length_of_straight;
			}

			// calculate the angle of the driving crank
			double angle = atan2(kinematics.diagram.joints[2]->pos.y - p0.y, kinematics.diagram.joints[2]->pos.x - p0.x);

			// update the lengths of the trajectory of the moving body
			std::vector<glm::dvec2> next_body_pts = kinematics.diagram.bodies[0]->getActualPoints();
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

}