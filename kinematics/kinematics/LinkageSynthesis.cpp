#include "LinkageSynthesis.h"

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
}