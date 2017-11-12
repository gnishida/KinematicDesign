#include "LinkageSynthesis.h"
#include <QFile>
#include <QTextStream>

namespace kinematics {

	bool LinkageSynthesis::compare(const std::pair<double, Solution>& s1, const std::pair<double, Solution>& s2) {
		return s1.first < s2.first;
	}

	/**
	 * Perturbe the poses a little based on the sigma.
	 */
	std::vector<glm::dmat3x3> LinkageSynthesis::perturbPoses(const std::vector<glm::dmat3x3>& poses, std::vector<std::pair<double, double>>& sigmas, double& position_error, double& orientation_error) {
		std::vector<glm::dmat3x3> perturbed_poses = poses;

		position_error = 0.0;
		orientation_error = 0.0;

		for (int i = 0; i < perturbed_poses.size(); i++) {
			double e1 = 0;
			double e2 = 0;
			double delta_theta = 0;
			if (i == 0) {	// first pose
				e1 = genNormal(0, sigmas[0].first);
				e2 = genNormal(0, sigmas[0].first);
				delta_theta = genNormal(0, sigmas[0].second);
			}
			else if (i == perturbed_poses.size() - 1) {	// last pose
				e1 = genNormal(0, sigmas[2].first);
				e2 = genNormal(0, sigmas[2].first);
				delta_theta = genNormal(0, sigmas[2].second);
			}
			else {	// poses in the middle
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

		return perturbed_poses;
	}

	/**
	 * Create a distance map for the linkage region.
	 */
	void LinkageSynthesis::createDistanceMapForLinkageRegion(const std::vector<glm::dvec2>& linkage_region_pts, double scale, BBox& dist_map_bbox, cv::Mat& dist_map) {
		// calculate the center of the linkage region
		BBox bbox = boundingBox(linkage_region_pts);
		glm::dvec2 center = bbox.center();

		// calculate the enlarged linkage region for the sampling region
		std::vector<glm::dvec2> enlarged_linkage_region_pts;
		for (int i = 0; i < linkage_region_pts.size(); i++) {
			enlarged_linkage_region_pts.push_back((linkage_region_pts[i] - center) * (double)scale + center);
		}

		// calculate the bounding box for the distance map
		dist_map_bbox = boundingBox(enlarged_linkage_region_pts);

		cv::Mat img(dist_map_bbox.height() + 1, dist_map_bbox.width() + 1, CV_8U, cv::Scalar(255));

		std::vector<std::vector<cv::Point>> pts(1);
		for (int i = 0; i < linkage_region_pts.size(); i++) {
			double x = linkage_region_pts[i].x - dist_map_bbox.minPt.x;
			double y = linkage_region_pts[i].y - dist_map_bbox.minPt.y;
			pts[0].push_back(cv::Point(x, y));
		}
		cv::fillPoly(img, pts, cv::Scalar(0), 4);
		
		cv::distanceTransform(img, dist_map, CV_DIST_L2, 3);
		//cv::imwrite("test2.png", img);
		//cv::imwrite("test.png", dist_map);

		// convert float type to double type
		dist_map.convertTo(dist_map, CV_64F);
	}

	void LinkageSynthesis::particleFilter(std::vector<Solution>& solutions, const std::vector<glm::dvec2>& linkage_region_pts, const cv::Mat& dist_map, const BBox& dist_map_bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, const std::vector<Object25D>& fixed_body_pts, const Object25D& body_pts, bool rotatable_crank, bool avoid_branch_defect, double min_link_length, double position_error_weight, double orientation_error_weight, double linkage_location_weight, double smoothness_weight, double size_weight, int num_particles, int num_iterations) {
		BBox linkage_region_bbox = boundingBox(linkage_region_pts);

		std::vector<std::pair<double, Solution>> particles(solutions.size());

		// sort the solutions by the cost
		for (int i = 0; i < solutions.size(); i++) {
			double cost = calculateCost(solutions[i], body_pts, dist_map, dist_map_bbox, position_error_weight, orientation_error_weight, linkage_location_weight, smoothness_weight, size_weight);
			particles[i] = std::make_pair(cost, solutions[i]);
		}
		std::sort(particles.begin(), particles.end(), compare);

		// select top num_particles solutions as the initial points
		if (particles.size() > num_particles) {
			particles.resize(num_particles);
		}
		else {
			// if the number of solutions is less than 100, augment the solutions to make 100 initial points
			int N = particles.size();
			particles.resize(num_particles);
			int cnt = 0;
			while (N + cnt < num_particles) {
				particles[N + cnt] = particles[cnt % N];
				cnt++;
			}
		}

		/*
		QFile file("particle_filter.txt");
		file.open(QIODevice::WriteOnly);
		QTextStream out(&file);
		double min_val = std::numeric_limits<double>::max();
		double max_val = 0;
		for (int i = 0; i < particles.size(); i++) {
			if (particles[i].first == std::numeric_limits<double>::max()) continue;

			min_val = std::min(min_val, particles[i].first);
			max_val = std::max(max_val, particles[i].first);
		}
		double avg_val = (min_val + max_val) * 0.5;
		out << min_val << "," << avg_val << "," << max_val << "\n";
		*/

		// particle filter
		for (int iter = 0; iter < num_iterations; iter++) {
			// perturb the particles and calculate its score
			std::vector<std::pair<double, Solution>> new_particles = particles;
			for (int i = 0; i < new_particles.size(); i++) {
				// pertube the joints
				for (int j = 0; j < new_particles[i].second.points.size(); j++) {
					new_particles[i].second.points[j].x += genRand(-1, 1);
					new_particles[i].second.points[j].y += genRand(-1, 1);
				}

				optimizeCandidate(new_particles[i].second.poses, linkage_region_pts, linkage_region_bbox, new_particles[i].second.points);

				// check the hard constraints
				if (checkHardConstraints(new_particles[i].second.points, new_particles[i].second.poses, linkage_region_pts, linkage_avoidance_pts, fixed_body_pts, body_pts, rotatable_crank, avoid_branch_defect, min_link_length, new_particles[i].second.zorder)) {
					// calculate the score
					double cost = calculateCost(new_particles[i].second, body_pts, dist_map, dist_map_bbox, position_error_weight, orientation_error_weight, linkage_location_weight, smoothness_weight, size_weight);
					new_particles[i] = std::make_pair(cost, new_particles[i].second);
				}
				else {
					// for the invalid point, make the cost infinity so that it will be discarded.
					new_particles[i] = std::make_pair(std::numeric_limits<double>::max(), new_particles[i].second);
				}
			}

			// merge the particles
			particles.insert(particles.end(), new_particles.begin(), new_particles.end());

			// take the top num_particles partciles
			std::sort(particles.begin(), particles.end(), compare);
			particles.resize(num_particles);

			/*
			min_val = std::numeric_limits<double>::max();
			max_val = 0;
			for (int i = 0; i < particles.size(); i++) {
				if (particles[i].first == std::numeric_limits<double>::max()) continue;

				min_val = std::min(min_val, particles[i].first);
				max_val = std::max(max_val, particles[i].first);
			}
			avg_val = (min_val + max_val) * 0.5;
			out << min_val << "," << avg_val << "," << max_val << "\n";
			*/
		}
		//file.close();

		// update solutions
		solutions.resize(particles.size());
		for (int i = 0; i < particles.size(); i++) {
			solutions[i] = particles[i].second;
		}
	}

}