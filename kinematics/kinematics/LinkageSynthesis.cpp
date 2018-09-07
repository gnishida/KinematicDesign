#include "LinkageSynthesis.h"
#include <QFile>
#include <QTextStream>

namespace kinematics {

	void LinkageSynthesis::calculateStatistics(const std::vector<double>& values, double& mean, double& sd) {
		double total = 0.0;
		for (int i = 0; i < values.size(); i++) {
			total += values[i];
		}
		mean = total / values.size();

		double var = 0.0;
		for (int i = 0; i < values.size(); i++) {
			var += (values[i] - mean) * (values[i] - mean);
		}
		sd = std::sqrt(var / values.size());
	}

	bool LinkageSynthesis::compare(const Solution& s1, const Solution& s2) {
		return s1.cost < s2.cost;
	}

	/**
	 * Perturbe the poses a little based on the sigma.
	 */
	std::vector<glm::dmat3x3> LinkageSynthesis::perturbPoses(const std::vector<glm::dmat3x3>& poses, std::pair<double, double>& sigmas, double& position_error, double& orientation_error) {
		std::vector<glm::dmat3x3> perturbed_poses = poses;

		position_error = 0.0;
		orientation_error = 0.0;

		for (int i = 1; i < perturbed_poses.size() - 1; i++) {
			double e1 = genNormal(0, sigmas.first);
			double e2 = genNormal(0, sigmas.first);
			double delta_theta = genNormal(0, sigmas.second);

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

	std::vector<glm::dvec2> LinkageSynthesis::enlargePolygon(const std::vector<glm::dvec2>& polygon, const glm::dvec2& center, double scale) {
		std::vector<glm::dvec2> result(polygon.size());
		for (int i = 0; i < polygon.size(); i++) {
			result[i] = (polygon[i] - center) * (double)scale + center;
		}

		return result;
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

	void LinkageSynthesis::particleFilter(std::vector<Solution>& solutions, const std::vector<glm::dvec2>& linkage_region_pts, const cv::Mat& dist_map, const BBox& dist_map_bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, const Object25D& moving_body, int num_particles, int num_iterations, bool record_file) {
		BBox linkage_region_bbox = boundingBox(linkage_region_pts);

		std::vector<Solution> particles(std::max((int)solutions.size(), num_particles));
		double max_cost = 0;

		/*
		for (int i = 0; i < solutions.size(); i++) {
			double cost = calculateCost(solutions[i], moving_body, dist_map, dist_map_bbox);
			max_cost = std::max(max_cost, cost);
			particles[i] = Particle(cost, solutions[i]);
		}
		*/

		// augment
		for (int i = 0; i < particles.size(); i++) {
			solutions[i % solutions.size()].cost = calculateCost(solutions[i % solutions.size()], moving_body, dist_map, dist_map_bbox);
			max_cost = std::max(max_cost, solutions[i % solutions.size()].cost);
			particles[i] = solutions[i % solutions.size()];
		}

		QFile* file;
		QTextStream* out;

		if (record_file) {
			file = new QFile("particle_filter.txt");
			file->open(QIODevice::WriteOnly);
			out = new QTextStream(file);

			std::vector<double> values;
			for (int i = 0; i < particles.size(); i++) {
				if (particles[i].cost == std::numeric_limits<double>::max()) continue;

				values.push_back(particles[i].cost);
			}
			double mean_val;
			double sd_val;
			calculateStatistics(values, mean_val, sd_val);
			(*out) << mean_val << "," << sd_val << "\n";
		}

		// particle filter
		for (int iter = 0; iter < num_iterations; iter++) {
			// perturb the particles and calculate its score
			std::vector<Solution> new_particles = particles;
			for (int i = 0; i < new_particles.size(); i++) {
				// pertube the joints
				for (int j = 0; j < new_particles[i].points.size(); j++) {
					new_particles[i].points[j].x += genRand(-1, 1);
					new_particles[i].points[j].y += genRand(-1, 1);
				}

				if (optimizeCandidate(new_particles[i].poses, linkage_region_pts, linkage_region_bbox, new_particles[i].points)) {
					// check the hard constraints
					if (checkHardConstraints(new_particles[i].points, new_particles[i].poses, linkage_region_pts, linkage_avoidance_pts, moving_body, new_particles[i].zorder)) {
						// calculate the score
						new_particles[i].cost = calculateCost(new_particles[i], moving_body, dist_map, dist_map_bbox);
					}
					else {
						// for the invalid point, make the cost infinity so that it will be discarded.
						new_particles[i].cost = std::numeric_limits<double>::max();
					}
				}
				else {
					// for the invalid point, make the cost infinity so that it will be discarded.
					new_particles[i].cost = std::numeric_limits<double>::max();
				}
			}

			// merge the particles
			new_particles.insert(new_particles.end(), particles.begin(), particles.end());

			// calculate the weights of particles
			resample(new_particles, num_particles, particles, max_cost);

			if (record_file) {
				std::vector<double> values;
				for (int i = 0; i < particles.size(); i++) {
					if (particles[i].cost == std::numeric_limits<double>::max()) continue;

					values.push_back(particles[i].cost);
				}
				double mean_val;
				double sd_val;
				calculateStatistics(values, mean_val, sd_val);
				(*out) << mean_val << "," << sd_val << "\n";
			}
		}

		if (record_file) {
			file->close();
			delete out;
			delete file;
		}

		// sort the particles based on the costs
		sort(particles.begin(), particles.end(), compare);

		// update solutions
		solutions = particles;
	}

	/**
	* Resample the particles based on their costs.
	*
	* @param particles				original set of particles
	* @param N						the number of new resampled particles
	* @param resampled_particles	new resmapled particles
	* @param max_cost				maximum cost, which is used to normalized the cost for calculating the weight
	*/
	void LinkageSynthesis::resample(std::vector<Solution> particles, int N, std::vector<Solution>& resampled_particles, double max_cost) {
		// calculate the weights of particles
		int best_index = -1;
		double min_cost = std::numeric_limits<double>::max();
		std::vector<double> weights(particles.size());
		double weight_total = 0.0;
		for (int i = 0; i < particles.size(); i++) {
			double w;
			if (particles[i].cost == std::numeric_limits<double>::max()) {
				w = 0;
			}
			else {
				w = std::exp(-particles[i].cost / max_cost * 20);
				if (particles[i].cost < min_cost) {
					min_cost = particles[i].cost;
					best_index = i;
				}
			}

			if (i == 0) {
				weights[i] = w;
			}
			else {
				weights[i] = weights[i - 1] + w;
			}
			weight_total += w;
		}
		for (int i = 0; i < particles.size(); i++) {
			weights[i] /= weight_total;
		}

		// resample the particles based on their weights
		resampled_particles.resize(N);
		resampled_particles[0] = particles[best_index];
		for (int i = 1; i < N; i++) {
			double r = genRand();
			auto it = std::lower_bound(weights.begin(), weights.end(), r);
			int index = it - weights.begin();
			resampled_particles[i] = particles[index];
		}
	}

}