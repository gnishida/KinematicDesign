#include "LeastSquareSolver.h"
#include "KinematicUtils.h"

namespace kinematics {

	SolverForLink::SolverForLink(const std::vector<glm::dmat3x3>& poses) {
		this->poses = poses;
	}

	double SolverForLink::operator() (const column_vector& arg) const {
		glm::dvec2 A0(arg(0, 0), arg(1, 0));
		glm::dvec2 a(arg(2, 0), arg(3, 0));

		glm::dvec2 A1(poses[0] * glm::dvec3(a, 1));
		double l1_squared = glm::length(A1 - A0);
		l1_squared = l1_squared * l1_squared;

		double ans = 0.0;
		for (int i = 1; i < poses.size(); i++) {
			glm::dvec2 A(poses[i] * glm::dvec3(a, 1));
			double l_squared = glm::length(A - A0);
			l_squared = l_squared * l_squared;
			ans += (l_squared - l1_squared) * (l_squared - l1_squared);
		}

		return ans;
	}

	SolverForSlider::SolverForSlider(const std::vector<glm::dmat3x3>& poses) {
		this->poses = poses;
	}

	double SolverForSlider::operator() (const column_vector& arg) const {
		glm::dvec2 a(arg(0, 0), arg(1, 0));

		glm::dvec2 A1(poses[0] * glm::dvec3(a, 1));
		glm::dvec2 A2(poses[1] * glm::dvec3(a, 1));

		glm::dvec2 v1 = A2 - A1;
		v1 /= glm::length(v1);

		double ans = 0.0;
		for (int i = 2; i < poses.size(); i++) {
			glm::dvec2 A(poses[i] * glm::dvec3(a, 1));
			glm::dvec2 v = A - A1;
			v /= glm::length(v);

			ans += abs(crossProduct(v1, v));
		}

		return ans;
	}

	SolverForWattI::SolverForWattI(const std::vector<std::vector<glm::dmat3x3>>& poses) {
		this->poses = poses;
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