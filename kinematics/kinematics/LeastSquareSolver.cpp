#include "LeastSquareSolver.h"
#include "KinematicUtils.h"

namespace kinematics {

	SolverForLink::SolverForLink(const std::vector<glm::dmat3x3>& poses) {
		pose_params.resize(poses.size() - 1, std::vector<double>(3));
		for (int i = 1; i < poses.size(); i++) {
			glm::dmat3x3 D = poses[i] * glm::inverse(poses[0]);
			pose_params[i - 1][0] = std::atan2(D[0][1], D[0][0]);
			pose_params[i - 1][1] = D[2][0];
			pose_params[i - 1][2] = D[2][1];
		}

		this->pose_params = pose_params;
	}

	double SolverForLink::operator() (const column_vector& arg) const {
		glm::dvec2 A(arg(0), arg(1));
		glm::dvec2 B(arg(2), arg(3));

		double ans = 0.0;
		for (int i = 0; i < pose_params.size(); i++) {
			double theta = pose_params[i][0];
			double u = pose_params[i][1];
			double v = pose_params[i][2];

			double z = 2 * (A.x * B.x + A.y * B.y) * (1 - std::cos(theta))
				+ 2 * (A.x * B.y - A.y * B.x) * std::sin(theta)
				- 2 * u * A.x
				- 2 * v * A.y
				+ 2 * B.x * (u * std::cos(theta) + v * std::sin(theta))
				+ 2 * B.y * (-u * std::sin(theta) + v * std::cos(theta))
				+ u * u
				+ v * v;

			ans += z * z;
		}

		return ans;
	}

	SolverDerivForLink::SolverDerivForLink(const std::vector<glm::dmat3x3>& poses) {
		pose_params.resize(poses.size() - 1, std::vector<double>(3));
		for (int i = 1; i < poses.size(); i++) {
			glm::dmat3x3 D = poses[i] * glm::inverse(poses[0]);
			pose_params[i - 1][0] = std::atan2(D[0][1], D[0][0]);
			pose_params[i - 1][1] = D[2][0];
			pose_params[i - 1][2] = D[2][1];
		}

		this->pose_params = pose_params;
	}

	const column_vector SolverDerivForLink::operator() (const column_vector& arg) const {
		glm::dvec2 A(arg(0), arg(1));
		glm::dvec2 B(arg(2), arg(3));

		column_vector ans(4);
		for (int i = 0; i < 4; i++) ans(i) = 0;

		for (int i = 0; i < pose_params.size(); i++) {
			double theta = pose_params[i][0];
			double u = pose_params[i][1];
			double v = pose_params[i][2];

			double z = 2 * (A.x * B.x + A.y * B.y) * (1 - std::cos(theta))
				+ 2 * (A.x * B.y - A.y * B.x) * std::sin(theta)
				- 2 * u * A.x
				- 2 * v * A.y
				+ 2 * B.x * (u * std::cos(theta) + v * std::sin(theta))
				+ 2 * B.y * (-u * std::sin(theta) + v * std::cos(theta))
				+ u * u
				+ v * v;

			ans(0) += 4 * z * (B.x * (1 - std::cos(theta)) + B.y * std::sin(theta) - u);
			ans(1) += 4 * z * (B.y * (1 - std::cos(theta)) - B.x * std::sin(theta) - v);
			ans(2) += 4 * z * (A.x * (1 - std::cos(theta)) - A.y * std::sin(theta) + u * std::cos(theta) + v * std::sin(theta));
			ans(3) += 4 * z * (A.y * (1 - std::cos(theta)) + A.x * std::sin(theta) - u * std::sin(theta) + v * std::cos(theta));
		}

		return ans;
	}

	SolverForSlider::SolverForSlider(const std::vector<glm::dmat3x3>& poses) {
		this->poses = poses;
		inv_pose0 = glm::inverse(poses[0]);
	}

	double SolverForSlider::operator() (const column_vector& arg) const {
		glm::dvec2 A0(arg(0, 0), arg(1, 0));
		glm::dvec2 A1(arg(2, 0), arg(3, 0));

		glm::dvec2 a(inv_pose0 * glm::dvec3(A1, 1));

		glm::dvec2 v1 = A0 - A1;
		v1 /= glm::length(v1);

		double ans = 0.0;
		for (int i = 1; i < poses.size(); i++) {
			glm::dvec2 A(poses[i] * glm::dvec3(a, 1));
			glm::dvec2 v = A - A1;
			v /= glm::length(v);
			double d = abs(crossProduct(v1, v));
			ans += d * d;
		}

		return ans;
	}

	SolverForWattI::SolverForWattI(const std::vector<glm::dmat3x3>& poses) {
		this->poses = poses;
		inv_pose0 = glm::inverse(poses[0]);
	}

	double SolverForWattI::operator() (const column_vector& arg) const {
		glm::dvec2 P0(arg(0, 0), arg(1, 0));
		glm::dvec2 P1(poses[0][0] * glm::dvec3(arg(2, 0), arg(3, 0), 1));
		glm::dvec2 P3(poses[0][0] * glm::dvec3(arg(6, 0), arg(7, 0), 1));
		glm::dvec2 P4(poses[0][0] * glm::dvec3(arg(8, 0), arg(9, 0), 1));
		glm::dvec2 P5(poses[1][0] * glm::dvec3(arg(10, 0), arg(11, 0), 1));
		glm::dvec2 P6(poses[1][0] * glm::dvec3(arg(12, 0), arg(13, 0), 1));

		glm::dmat3x3 T = calculateTransMatrix(P3, P5);
		glm::dvec2 P2(T * glm::dvec3(arg(4, 0), arg(5, 0), 1));

		std::vector<double> lengths;
		lengths.push_back(P1.x * P1.x);
		lengths.push_back(P1.y * P1.y);
		lengths.push_back(glm::length2(P2 - P0));
		//lengths.push_back(glm::length2(P5 - P2));
		lengths.push_back(glm::length2(P5 - P3));
		lengths.push_back(glm::length2(P6 - P4));

		double ans = 0.0;
		for (int i = 1; i < poses.size(); i++) {
			glm::dvec2 P1b(poses[0][i] * glm::dvec3(arg(2, 0), arg(3, 0), 1));
			glm::dvec2 P3b(poses[0][i] * glm::dvec3(arg(6, 0), arg(7, 0), 1));
			glm::dvec2 P4b(poses[0][i] * glm::dvec3(arg(8, 0), arg(9, 0), 1));
			glm::dvec2 P5b(poses[1][i] * glm::dvec3(arg(10, 0), arg(11, 0), 1));
			glm::dvec2 P6b(poses[1][i] * glm::dvec3(arg(12, 0), arg(13, 0), 1));

			glm::dmat3x3 Tb = calculateTransMatrix(P3b, P5b);
			glm::dvec2 P2b(Tb * glm::dvec3(arg(4, 0), arg(5, 0), 1));

			std::vector<double> lengths2;
			lengths2.push_back(P1b.x * P1b.x);
			lengths2.push_back(P1b.y * P1b.y);
			lengths2.push_back(glm::length2(P2b - P0));
			//lengths2.push_back(glm::length2(P5b - P2b));
			lengths2.push_back(glm::length2(P5b - P3b));
			lengths2.push_back(glm::length2(P6b - P4b));

			for (int i = 0; i < lengths.size(); i++) {
				ans += (lengths[i] - lengths2[i]) * (lengths[i] - lengths2[i]);
			}
		}

		return ans;
	}

}