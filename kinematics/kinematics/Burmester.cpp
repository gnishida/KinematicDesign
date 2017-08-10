#include "Burmester.h"
#include "KinematicUtils.h"

namespace kinematics {

	bool compare(const SpecialPoint& p1, const SpecialPoint& p2) {
		return p1.index < p2.index;
	}

	void calculateSolutionCurve(const std::vector<glm::dmat4x4>& poses, std::vector<std::vector<std::vector<glm::dvec2>>>& solutions) {
		// calculate the coordinates of two points on the coupler
		std::vector<std::vector<glm::dvec2>> points(poses.size());
		for (int i = 0; i < poses.size(); i++) {
			glm::dvec2 a(poses[i] * glm::dvec4(0, 0, 0, 1));
			glm::dvec2 b(poses[i] * glm::dvec4(0.843, 0, 0, 1));

			points[i].push_back(a);
			points[i].push_back(b);
		}

		// calculate the coordinates of pole_{i,j}
		std::vector<std::vector<std::vector<glm::dvec2>>> P = calculatePoles(poses);
		
		// calculate theta_{12} and theta_{13}
		double theta1 = M_PI;
		if (poses[0][0][0] != 0.0) {
			theta1 = atan2(poses[0][0][1], poses[0][0][0]);
		}
		double theta2 = M_PI;
		if (poses[1][0][0] != 0.0) {
			theta2 = atan2(poses[1][0][1], poses[1][0][0]);
		}
		double theta3 = M_PI;
		if (poses[2][0][0] != 0.0) {
			theta3 = atan2(poses[2][0][1], poses[2][0][0]);
		}
		double theta12 = theta1 - theta2;
		double theta13 = theta1 - theta3;

		solutions = calculateCenterPointCurve(P[0][0][1], P[0][0][2], P[0][0][3], P[0][1][2], P[0][1][3], theta12, theta13);
	}

	/**
	 * Calculate the center point curve for the opposite pole quadrilateral, P_{13}, P_{14}, P_{24}, and P_{23}
	 */
	std::vector<std::vector<std::vector<glm::dvec2>>> calculateCenterPointCurve(const glm::dvec2& P12, const glm::dvec2& P13, const glm::dvec2& P14, const glm::dvec2& P23, const glm::dvec2& P24, double theta12, double theta13) {
		std::vector<std::vector<std::vector<glm::dvec2>>> curves(2);

		// calculate the mid points of P13 and P14, P23 and P24
		glm::dvec2 m1 = (P13 + P14) * 0.5;
		glm::dvec2 m2 = (P24 + P23) * 0.5;

		// calculate the normal vector
		glm::dvec2 v1 = P13 - P14;
		v1 /= glm::length(v1);
		glm::dvec2 h1(-v1.y, v1.x);
		glm::dvec2 v2 = P23 - P24;
		v2 /= glm::length(v2);
		glm::dvec2 h2(-v2.y, v2.x);

		// calculate the circle points
		bool inloop = false;
		std::vector<std::vector<glm::dvec2>> center_loops(2);
		std::vector<std::vector<glm::dvec2>> circle_loops(2);
		for (double alpha = -kinematics::M_PI * 0.5 + 0.001; alpha <= kinematics::M_PI * 0.5 - 0.001; alpha += 0.001) {
			glm::dvec2 u1(cos(alpha) * v1.x - sin(alpha) * v1.y, sin(alpha) * v1.x + cos(alpha) * v1.y);
			glm::dvec2 M1;
			kinematics::lineLineIntersection(m1, h1, P14, u1, M1);
			double r1 = glm::length(P14 - M1);

			glm::dvec2 u2(cos(alpha) * v2.x - sin(alpha) * v2.y, sin(alpha) * v2.x + cos(alpha) * v2.y);
			glm::dvec2 M2;
			kinematics::lineLineIntersection(m2, h2, P24, u2, M2);
			double r2 = glm::length(P24 - M2);

			glm::dvec2 C1, C2;
			try {
				C1 = kinematics::circleCircleIntersection(M1, r1, M2, r2);
				C2 = kinematics::circleCircleIntersection(M2, r2, M1, r1);

				// calculate the corresponding circle point
				if (glm::length(C1 - glm::dvec2(3.65, 3.9)) < 0.1 || glm::length(C2 - glm::dvec2(3.65, 3.9) )< 0.1) {
					int hoge = 0;
				}
				glm::dvec2 circle_pt1 =	calculateCirclePointFromCenterPoint(C1, P12, P13, theta12, theta13);
				glm::dvec2 circle_pt2 = calculateCirclePointFromCenterPoint(C2, P12, P13, theta12, theta13);

				if (!inloop) {
					inloop = true;
				}

				if (center_loops[0].size() == 0 || glm::length(center_loops[0].back() - C1) > 0.1 || glm::length(center_loops[1].back() - C2) > 0.1 || glm::length(circle_loops[0].back() - circle_pt1) > 0.1 || glm::length(circle_loops[1].back() - circle_pt2) > 0.1) {
					center_loops[0].push_back(C1);
					center_loops[1].push_back(C2);
					circle_loops[0].push_back(circle_pt1);
					circle_loops[1].push_back(circle_pt2);
				}
			}
			catch (char* ex) {
				if (center_loops[0].size() > 0) {
					curves[0].resize(curves[0].size() + 1);
					curves[1].resize(curves[1].size() + 1);

					double d1 = glm::length(center_loops[0].front() - center_loops[1].front());
					double d2 = glm::length(circle_loops[0].front() - circle_loops[1].front());
					double d3 = glm::length(center_loops[0].back() - center_loops[1].back());
					double d4 = glm::length(circle_loops[0].back() - circle_loops[1].back());
					if ((d1 <= d2 && d1 <= d3 && d1 <= d4) || (d2 <= d3 && d2 <= d4)) {
						std::reverse(center_loops[0].begin(), center_loops[0].end());
						std::reverse(circle_loops[0].begin(), circle_loops[0].end());
					}
					else {
						std::reverse(center_loops[1].begin(), center_loops[1].end());
						std::reverse(circle_loops[1].begin(), circle_loops[1].end());
					}

					curves[0].back().insert(curves[0].back().end(), center_loops[0].begin(), center_loops[0].end());
					curves[0].back().insert(curves[0].back().end(), center_loops[1].begin(), center_loops[1].end());
					curves[1].back().insert(curves[1].back().end(), circle_loops[0].begin(), circle_loops[0].end());
					curves[1].back().insert(curves[1].back().end(), circle_loops[1].begin(), circle_loops[1].end());

					center_loops.clear();
					center_loops.resize(2);
					circle_loops.clear();
					circle_loops.resize(2);
				}
				
				inloop = false;
			}
		}

		if (center_loops[0].size() > 0) {
			curves[0].resize(curves[0].size() + 1);
			curves[1].resize(curves[1].size() + 1);

			double d1 = glm::length(center_loops[0].front() - center_loops[1].front());
			double d2 = glm::length(circle_loops[0].front() - circle_loops[1].front());
			double d3 = glm::length(center_loops[0].back() - center_loops[1].back());
			double d4 = glm::length(circle_loops[0].back() - circle_loops[1].back());
			if ((d1 <= d2 && d1 <= d3 && d1 <= d4) || (d2 <= d3 && d2 <= d4)) {
				std::reverse(center_loops[0].begin(), center_loops[0].end());
				std::reverse(circle_loops[0].begin(), circle_loops[0].end());
			}
			else {
				std::reverse(center_loops[1].begin(), center_loops[1].end());
				std::reverse(circle_loops[1].begin(), circle_loops[1].end());
			}

			curves[0].back().insert(curves[0].back().end(), center_loops[0].begin(), center_loops[0].end());
			curves[0].back().insert(curves[0].back().end(), center_loops[1].begin(), center_loops[1].end());
			curves[1].back().insert(curves[1].back().end(), circle_loops[0].begin(), circle_loops[0].end());
			curves[1].back().insert(curves[1].back().end(), circle_loops[1].begin(), circle_loops[1].end());
		}

		// merge the loops that are connected
		for (int i = 0; i < curves[1].size() - 1; i++) {
			for (int j = curves[1].size() - 1; j > i; j--) {
				if (glm::length(curves[1][i].front() - curves[1][j].front()) < 0.1) {
					std::reverse(curves[0][j].begin(), curves[0][j].end());
					std::reverse(curves[1][j].begin(), curves[1][j].end());
					curves[0][i].insert(curves[0][i].begin(), curves[0][j].begin(), curves[0][j].end());
					curves[1][i].insert(curves[1][i].begin(), curves[1][j].begin(), curves[1][j].end());
					curves[0].erase(curves[0].begin() + j);
					curves[1].erase(curves[1].begin() + j);
				}
				else if (glm::length(curves[1][i].back() - curves[1][j].back()) < 0.1) {
					std::reverse(curves[0][j].begin(), curves[0][j].end());
					std::reverse(curves[1][j].begin(), curves[1][j].end());
					curves[0][i].insert(curves[0][i].end(), curves[0][j].begin(), curves[0][j].end());
					curves[1][i].insert(curves[1][i].end(), curves[1][j].begin(), curves[1][j].end());
					curves[0].erase(curves[0].begin() + j);
					curves[1].erase(curves[1].begin() + j);
				}
				else if (glm::length(curves[1][i].front() - curves[1][j].back()) < 0.1) {
					curves[0][i].insert(curves[0][i].begin(), curves[0][j].begin(), curves[0][j].end());
					curves[1][i].insert(curves[1][i].begin(), curves[1][j].begin(), curves[1][j].end());
					curves[0].erase(curves[0].begin() + j);
					curves[1].erase(curves[1].begin() + j);
				}
				else if (glm::length(curves[1][i].back() - curves[1][j].front()) < 0.1) {
					curves[0][i].insert(curves[0][i].end(), curves[0][j].begin(), curves[0][j].end());
					curves[1][i].insert(curves[1][i].end(), curves[1][j].begin(), curves[1][j].end());
					curves[0].erase(curves[0].begin() + j);
					curves[1].erase(curves[1].begin() + j);
				}
			}
		}

		// Reorder the open loop such that it starts at infinity and ends at infinity of the other side.
		// If the loop is closed, the corresponding circle point curve is an open loop, so reorder the loop in a similar manner.
		for (int i = 0; i < curves[0].size(); i++) {
			double max_dist = 0;
			int max_j = -1;
			int n = curves[0][i].size();
			for (int j = 0; j < n; j++) {
				int next = (j + 1) % n;
				double dist = glm::length(curves[0][i][j] - curves[0][i][next]);
				if (dist > max_dist) {
					max_dist = dist;
					max_j = j;
				}
			}

			if (max_dist > 20) {
				curves[0][i].insert(curves[0][i].begin(), curves[0][i].begin() + max_j + 1, curves[0][i].end());
				curves[0][i].resize(n);
				curves[1][i].insert(curves[1][i].begin(), curves[1][i].begin() + max_j + 1, curves[1][i].end());
				curves[1][i].resize(n);
			}
			else {
				double max_dist = 0;
				int max_j = -1;
				int n = curves[1][i].size();
				for (int j = 0; j < n - 1; j++) {
					double dist = glm::length(curves[1][i][j] - curves[1][i][j + 1]);
					if (dist > max_dist) {
						max_dist = dist;
						max_j = j;
					}
				}

				if (max_dist > 20) {
					curves[0][i].insert(curves[0][i].begin(), curves[0][i].begin() + max_j + 1, curves[0][i].end());
					curves[0][i].resize(n);
					curves[1][i].insert(curves[1][i].begin(), curves[1][i].begin() + max_j + 1, curves[1][i].end());
					curves[1][i].resize(n);
				}
			}
		}

		return curves;
	}

	glm::dvec2 calculateCirclePointFromCenterPoint(const glm::dvec2& C, const glm::dvec2& P12, const glm::dvec2& P13, double theta12, double theta13) {
		glm::dvec2 s1 = C - P12;
		s1 /= glm::length(s1);
		s1 = glm::dvec2(cos(theta12 * 0.5) * s1.x - sin(theta12 * 0.5) * s1.y, sin(theta12 * 0.5) * s1.x + cos(theta12 * 0.5) * s1.y);
		glm::dvec2 s2 = C - P13;
		s2 /= glm::length(s2);
		s2 = glm::dvec2(cos(theta13 * 0.5) * s2.x - sin(theta13 * 0.5) * s2.y, sin(theta13 * 0.5) * s2.x + cos(theta13 * 0.5) * s2.y);

		glm::dvec2 circle_pt;
		if (!lineLineIntersection(P12, s1, P13, s2, circle_pt)) throw "No circle point";

		return circle_pt;
	}

	/**
	 * Calculate the poles, P12, P13, P14, P23, P24, P34, and image poles, P23', P24', P34'
	 */
	std::vector<std::vector<std::vector<glm::dvec2>>> calculatePoles(const std::vector<glm::dmat4x4>& poses) {
		// calculate the coordinates of two points on the coupler
		std::vector<std::vector<glm::dvec2>> points(poses.size());
		for (int i = 0; i < poses.size(); i++) {
			glm::dvec2 a(poses[i] * glm::dvec4(0, 0, 0, 1));
			glm::dvec2 b(poses[i] * glm::dvec4(0.843, 0, 0, 1));

			points[i].push_back(a);
			points[i].push_back(b);
		}

		// calculate the coordinates of pole_{i,j}
		std::vector<std::vector<std::vector<glm::dvec2>>> P(2, std::vector<std::vector<glm::dvec2>>(4, std::vector<glm::dvec2>(4)));
		int cnt = 0;
		for (int i = 0; i < points.size() - 1; i++) {
			for (int j = i + 1; j < points.size(); j++, cnt++) {
				glm::dvec2 m1 = (points[i][0] + points[j][0]) * 0.5;
				glm::dvec2 v1 = points[j][0] - points[i][0];
				glm::dvec2 n1(-v1.y, v1.x);
				n1 /= glm::length(n1);

				glm::dvec2 m2 = (points[i][1] + points[j][1]) * 0.5;
				glm::dvec2 v2 = points[j][1] - points[i][1];
				glm::dvec2 n2(-v2.y, v2.x);
				n2 /= glm::length(n2);

				glm::dvec2 pole;
				if (kinematics::lineLineIntersection(m1, n1, m2, n2, pole)) {
					P[0][i][j] = pole;
					P[0][j][i] = pole;
				}
			}
		}

		P[1] = P[0];

		// calculate the image poles, P23', P24', P34'
		P[1][1][2] = kinematics::reflect(P[0][1][2], P[0][0][1], P[0][0][1] - P[0][0][2]);
		P[1][1][3] = kinematics::reflect(P[0][1][3], P[0][0][1], P[0][0][1] - P[0][0][3]);
		P[1][2][3] = kinematics::reflect(P[0][2][3], P[0][0][2], P[0][0][2] - P[0][0][3]);
		P[1][2][1] = P[1][1][2];
		P[1][3][1] = P[1][1][3];
		P[1][3][2] = P[1][2][3];

		return P;
	}

	std::vector<std::vector<std::vector<SpecialPoint>>> calculatePoleIntersections(const std::vector<glm::dmat4x4>& poses, const std::vector<std::vector<std::vector<glm::dvec2>>>& curves) {
		std::vector<std::vector<std::vector<SpecialPoint>>> ans(2, std::vector<std::vector<SpecialPoint>>(curves[0].size()));

		std::vector<std::vector<std::vector<glm::dvec2>>> P = calculatePoles(poses);

		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 3; j++) {
				for (int k = j + 1; k < 4; k++) {
					std::vector<glm::dvec2> p;
					for (int l = 0; l < 4; l++) {
						if (l == j || l == k) continue;
						p.push_back(P[i][l][j]);
						p.push_back(P[i][l][k]);
					}

					glm::dvec2 Q;
					lineLineIntersection(p[0], p[0] - p[1], p[2], p[2] - p[3], Q);
					std::pair<int, int> indices = findSolution(curves[i], Q);
					ans[i][indices.first].push_back(SpecialPoint(indices.second, SpecialPoint::TYPE_Q, { j, k }));
				}
			}
		}

		return ans;
	}

	/**
	 * Given the circle point curve and poles, find Us and Ts.
	 */
	std::vector<std::vector<SpecialPoint>> calculateUTs(const std::vector<std::vector<glm::dvec2>>& curve, const std::vector<std::vector<std::vector<glm::dvec2>>>& P) {
		std::vector<std::vector<SpecialPoint>> ans(curve.size());

		for (int i = 0; i < curve.size(); i++) {
			for (int j = 0; j < 3; j++) {
				for (int k = j + 1; k < 4; k++) {
					int l = 0;
					for (; l < 4; l++) {
						if (l != j && l != k) break;
					}
					
					calculateUT(curve[i], P[1][l][j], P[1][l][k], { j, k }, ans[i]);
				}
			}
			std::sort(ans[i].begin(), ans[i].end(), compare);
		}

		return ans;
	}

	/**
	* Given the circle point curve and a pair of poles, find U and T.
	*/
	void calculateUT(const std::vector<glm::dvec2>& curve, const glm::dvec2& P1, const glm::dvec2& P2, const std::pair<int, int>& subscript, std::vector<SpecialPoint>& ret) {
		glm::dvec2 c = (P1 + P2) * 0.5;
		double r = glm::length(P2 - P1) * 0.5;

		for (int j = 0; j < curve.size() - 1; j++) {
			if (curve[j] == P1 && curve[j] == P2) continue;

			double d1 = glm::length(curve[j] - c);
			double d2 = glm::length(curve[j + 1] - c);
			if ((d1 < r && d2 > r) || (d1 > r && d2 < r)) {
				double s1 = abs(r - d1);
				double s2 = abs(r - d2);
				glm::dvec2 pt = curve[j] + (curve[j + 1] - curve[j]) / (s1 + s2) * s1;
				if (glm::length(pt - P1) > 0.1 && glm::length(pt - P2) > 0.1) {
					int index = findSolution(curve, pt);

					ret.push_back(SpecialPoint(index, SpecialPoint::TYPE_UT, subscript));
				}
			}
		}
	}

	/**
	 * Given a circle point curve, find the solution set that is permissible as the circle point of a driven crank without any branch defect
	 */
	std::vector<std::vector<std::tuple<int, int, int>>> findExtremePoses(const std::vector<glm::dmat4x4>& poses, const std::vector<std::vector<glm::dvec2>>& curve, std::vector<std::vector<glm::dvec2>>& P, std::vector<std::vector<SpecialPoint>>& Q, std::vector<std::vector<SpecialPoint>>& UT) {
		std::vector<std::vector<std::tuple<int, int, int>>> extreme_poses(curve.size());

		// calculate theta_i
		std::vector<double> theta(4, M_PI);
		if (poses[0][0][0] != 0.0) {
			theta[0] = atan2(poses[0][0][1], poses[0][0][0]);
		}
		double theta2 = M_PI;
		if (poses[1][0][0] != 0.0) {
			theta[1] = atan2(poses[1][0][1], poses[1][0][0]);
		}
		double theta3 = M_PI;
		if (poses[2][0][0] != 0.0) {
			theta[2] = atan2(poses[2][0][1], poses[2][0][0]);
		}
		double theta4 = M_PI;
		if (poses[3][0][0] != 0.0) {
			theta[3] = atan2(poses[3][0][1], poses[3][0][0]);
		}

		std::vector<std::vector<SpecialPoint>> special_points(curve.size());

		for (int i = 0; i < Q.size(); i++) {
			std::vector<SpecialPoint> pts = Q[i];
			pts.insert(pts.end(), UT[i].begin(), UT[i].end());

			std::sort(pts.begin(), pts.end(), compare);

			// initialize the sign of the six angles
			std::vector<std::vector<bool>> table(4, std::vector<bool>(4, false));
			if (glm::length(curve[i].front() - curve[i].back()) < 10.0) {
				// initialize the sign based on point B
				for (int j = 0; j < 4; j++) {
					for (int k = 0; k < 4; k++) {
						if (j == k) continue;
						if (theta[j] > theta[k]) {
							table[j][k] = true;
						}
						else {
							table[j][k] = false;
						}
					}
				}
			}
			else {
				// initialize the sign based on point S
				for (int j = 0; j < 4; j++) {
					for (int k = 0; k < 4; k++) {
						if (j == k) continue;
						int l = 0;
						for (; l < 4; l++) {
							if (l != j && l != k) break;
						}

						glm::dvec2 v1 = P[l][k] - P[l][j];
						glm::dvec2 v2 = curve[i].front() - P[l][j];
						if (v1.x * v2.y - v1.y * v2.x >= 0) {
							table[j][k] = true;
						}
						else {
							table[j][k] = false;
						}
					}
				}
			}

			// check the extreme poses
			std::pair<int, int> ep = findExtremePoses(table);
			extreme_poses[i].push_back(std::make_tuple(0, ep.first, ep.second));

			for (int j = 0; j < pts.size(); j++) {
				std::swap(table[pts[j].subscript.first][pts[j].subscript.second], table[pts[j].subscript.second][pts[j].subscript.first]);
				std::pair<int, int> ep = findExtremePoses(table);
				extreme_poses[i].push_back(std::make_tuple(pts[j].index, ep.first, ep.second));
			}
		}

		return extreme_poses;
	}

	std::pair<int, int> findExtremePoses(const std::vector<std::vector<bool>>& table) {
		int pose1 = -1;
		int pose2 = -1;
		for (int j = 0; j < 4; j++) {
			bool extreme = true;
			for (int k = 0; k < 4; k++) {
				if (j == k) continue;
				if (!table[j][k]) {
					extreme = false;
					break;
				}
			}

			if (extreme) {
				pose1 = j;
				break;
			}
		}

		for (int j = 0; j < 4; j++) {
			bool extreme = true;
			for (int k = 0; k < 4; k++) {
				if (j == k) continue;
				if (!table[k][j]) {
					extreme = false;
					break;
				}
			}

			if (extreme) {
				pose2 = j;
				break;
			}
		}

		return std::make_pair(pose1, pose2);
	}

	std::pair<int, int> findSolution(const std::vector<std::vector<glm::dvec2>>& curves, const glm::dvec2& pt) {
		std::pair<int, int> ans = { -1, -1 };

		double min_dist = std::numeric_limits<double>::max();
		for (int i = 0; i < curves.size(); i++) {
			for (int j = 0; j < curves[i].size(); j++) {
				double dist = glm::length(curves[i][j] - pt);
				if (dist < min_dist) {
					min_dist = dist;
					ans = { i, j };
				}
			}
		}

		return ans;
	}

	int findSolution(const std::vector<glm::dvec2>& curve, const glm::dvec2& pt) {
		int ans = -1;

		double min_dist = std::numeric_limits<double>::max();
		for (int i = 0; i < curve.size(); i++) {
			double dist = glm::length(curve[i] - pt);
			if (dist < min_dist) {
				min_dist = dist;
				ans = i;
			}
		}

		return ans;
	}

	std::vector<std::vector<glm::dvec2>> findValidSolution(const std::vector<glm::dmat4x4>& poses, const std::vector<std::vector<std::vector<glm::dvec2>>>& curves) {
		std::map<double, std::tuple<glm::dvec2, glm::dvec2, glm::dvec2, glm::dvec2>> solutions;

		for (int i = 0; i < curves[0].size(); i++) {
			for (int j = 0; j < curves[0][i].size(); j++) {
				for (int k = 0; k < curves[0].size(); k++) {
					for (int l = 0; l < curves[0][k].size(); l++) {
						if (i == k && j == l) continue;

						// get the coordinates of the input crank
						glm::dvec2 C1 = curves[0][i][j];
						glm::dvec2 X1 = curves[1][i][j];

						// get the coordinates of the follower crank
						glm::dvec2 C2 = curves[0][k][l];
						glm::dvec2 X2 = curves[1][k][l];

						double g = glm::length(C1 - C2);
						double a = glm::length(X1 - C1);
						double b = glm::length(X2 - C2);
						double h = glm::length(X1 - X2);
						if (g < 0.5 || a < 0.5 || b < 0.5 || h < 0.5) continue;

						if (checkGrashofDefect(C1, C2, X1, X2)) continue;
						if (checkOrderDefect(poses, C1, C2, X1, X2)) continue;
						if (checkBranchDefect(poses, C1, C2, X1, X2)) continue;

						solutions[g + a + b + h] = std::make_tuple(C1, C2, X1, X2);
					}
				}
			}
		}

		std::vector<std::vector<glm::dvec2>> ans(2, std::vector<glm::dvec2>(2));
		if (solutions.size() == 0) {
			std::cout << "No solution was found." << std::endl;
		}
		else {
			auto it = solutions.begin();
			ans[0][0] = std::get<0>(it->second);
			ans[1][0] = std::get<2>(it->second);
			ans[0][1] = std::get<1>(it->second);
			ans[1][1] = std::get<3>(it->second);
		}

		return ans;
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
	int getGrashofType(const glm::dvec2& C1, const glm::dvec2& C2, const glm::dvec2& X1, const glm::dvec2& X2) {
		double g = glm::length(C1 - C2);
		double a = glm::length(X1 - C1);
		double b = glm::length(X2 - C2);
		double h = glm::length(X1 - X2);

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
	* Check if the linkage has Grashof defect.
	* If the following conditions are not satisified, the linkage has Grashof defect, and true is returned.
	* - The sum of the shortest and longest link is less than the sum of the remaining links (i.e., s + l <= p + q).
	* - The shortest link is either a driving link or a ground link.
	* If both conditions are satisfied, there is no Grashof defect, and false is returned.
	*/
	bool checkGrashofDefect(const glm::dvec2& C1, const glm::dvec2& C2, const glm::dvec2& X1, const glm::dvec2& X2) {
		int linkage_type = getGrashofType(C1, C2, X1, X2);

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
	bool checkOrderDefect(const std::vector<glm::dmat4x4>& poses, const glm::dvec2& C1, const glm::dvec2& C2, const glm::dvec2& X1, const glm::dvec2& X2) {
		int linkage_type = getGrashofType(C1, C2, X1, X2);

		glm::dvec2 inv_W = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(X1, 0, 1));

		double total_cw = 0;
		double total_ccw = 0;
		double prev = 0;
		//int ccw = 1;
		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving crank in the world coordinate system
			glm::dvec2 X = glm::dvec2(poses[i] * glm::dvec4(inv_W, 0, 1));
			//std::cout << X.x << "," << X.y << std::endl;

			// calculate the direction from the ground pivot (center point) of the driving crank to the circle point
			glm::dvec2 dir = X - C1;

			// calculate its angle
			double theta = atan2(dir.y, dir.x);

			if (i >= 1) {
				if (theta >= prev) {
					total_cw += kinematics::M_PI * 2 - theta + prev;
					total_ccw += theta - prev;
				}
				else {
					total_cw += prev - theta;
					total_ccw += kinematics::M_PI * 2 - prev + theta;
				}
			}

			prev = theta;
		}

		if (total_cw > kinematics::M_PI * 2 + 0.1 && total_ccw > kinematics::M_PI * 2 + 0.1) return true;
		else return false;
	}

	/**
	* Check if all the poses are in the same branch.
	* If there is an branch defect, true is returned.
	* Otherwise, false is returned.
	*/
	bool checkBranchDefect(const std::vector<glm::dmat4x4>& poses, const glm::dvec2& C1, const glm::dvec2& C2, const glm::dvec2& X1, const glm::dvec2& X2) {
		int type = getGrashofType(C1, C2, X1, X2);

		if (type == 0) {	// Grashof (Drag-link)
			int sign1 = 1;
			int sign2 = 1;

			glm::dvec2 inv_W1 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(X1, 0, 1));
			glm::dvec2 inv_W2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(X2, 0, 1));

			for (int i = 0; i < poses.size(); i++) {
				// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
				glm::dvec2 X1 = glm::dvec2(poses[i] * glm::dvec4(inv_W1, 0, 1));
				glm::dvec2 X2 = glm::dvec2(poses[i] * glm::dvec4(inv_W2, 0, 1));

				// calculate the direction from the ground pivot (center point) of the driving crank to the circle point
				glm::dvec2 v1 = X1 - C1;

				// calculate the direction from the ground pivot (center point) of the driven crank to the circle point
				glm::dvec2 v2 = X2 - C2;

				// calculate the direction from the circle point of the driven crank to the circle point of the driving crank
				glm::dvec2 v3 = X1 - X2;

				// calculate its sign
				if (i == 0) {
					sign1 = (v1.x * v3.y - v1.y * v3.x >= 0) ? 1 : -1;
					sign2 = (v2.x * v3.y - v2.y * v3.x >= 0) ? 1 : -1;
				}
				else {
					/*
					if ((v1.x * v3.y - v1.y * v3.x >= 0 ? 1 : -1) != sign1) sign1 = 9999;
					if ((v2.x * v3.y - v2.y * v3.x >= 0 ? 1 : -1) != sign2) sign2 = 9999;
					if (sign1 == 9999 && sign2 == 9999) return true;
					*/
					if ((v1.x * v3.y - v1.y * v3.x >= 0 ? 1 : -1) != sign1) return true;
					if ((v2.x * v3.y - v2.y * v3.x >= 0 ? 1 : -1) != sign2) return true;
				}
			}

			return false;
		}
		else if (type == 1) {	// Grashof (Crank-rocker)
			int sign = 1;

			glm::dvec2 inv_W1 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(X1, 0, 1));
			glm::dvec2 inv_W2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(X2, 0, 1));

			for (int i = 0; i < poses.size(); i++) {
				// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
				glm::dvec2 X1 = glm::dvec2(poses[i] * glm::dvec4(inv_W1, 0, 1));
				glm::dvec2 X2 = glm::dvec2(poses[i] * glm::dvec4(inv_W2, 0, 1));

				// calculate the direction from the ground pivot (center point) of the driven crank to the circle point
				glm::dvec2 v1 = X2 - C2;

				// calculate the direction from the circle point of the driven crank to the circle point of the driving crank
				glm::dvec2 v2 = X1 - X2;

				// calculate its sign
				if (i == 0) {
					sign = v1.x * v2.y - v1.y * v2.x >= 0 ? 1 : -1;
				}
				else {
					if ((v1.x * v2.y - v1.y * v2.x >= 0 ? 1 : -1) != sign) return true;
				}
			}

			return false;
		}
		else if (type == 2 || type == 3) {	// Grashof (Rocker-crank or Double rocker)
			int sign1 = 1;
			int sign2 = 1;

			glm::dvec2 inv_W1 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(X1, 0, 1));
			glm::dvec2 inv_W2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(X2, 0, 1));

			for (int i = 0; i < poses.size(); i++) {
				// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
				glm::dvec2 X1 = glm::dvec2(poses[i] * glm::dvec4(inv_W1, 0, 1));
				glm::dvec2 X2 = glm::dvec2(poses[i] * glm::dvec4(inv_W2, 0, 1));

				// calculate the direction from the ground pivot (center point) of the driving crank to the circle point
				glm::dvec2 v1 = X1 - C1;

				// calculate the direction from the ground pivot (center point) of the driven crank to the circle point
				glm::dvec2 v2 = X2 - C2;

				// calculate the direction from the center point of the driven crank to the center point of the driving crank
				glm::dvec2 v3 = C1 - C2;

				// calculate its sign
				if (i == 0) {
					sign1 = (v1.x * v3.y - v1.y * v3.x >= 0) ? 1 : -1;
					sign2 = (v2.x * v3.y - v2.y * v3.x >= 0) ? 1 : -1;
				}
				else {
					if ((v1.x * v3.y - v1.y * v3.x >= 0 ? 1 : -1) != sign1) return true;
					if ((v2.x * v3.y - v2.y * v3.x >= 0 ? 1 : -1) != sign2) return true;
				}
			}

			return false;
		}
		else {	// Non-Grashof
			int sign1 = 1;
			int sign2 = 1;

			glm::dvec2 inv_W1 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(X1, 0, 1));
			glm::dvec2 inv_W2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(X2, 0, 1));

			for (int i = 0; i < poses.size(); i++) {
				// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
				glm::dvec2 X1 = glm::dvec2(poses[i] * glm::dvec4(inv_W1, 0, 1));
				glm::dvec2 X2 = glm::dvec2(poses[i] * glm::dvec4(inv_W2, 0, 1));

				// calculate the direction from the ground pivot (center point) of the driving crank to the circle point
				glm::dvec2 v1 = X1 - C1;

				// calculate the direction from the ground pivot (center point) of the driven crank to the circle point
				glm::dvec2 v2 = X2 - C2;

				// calculate the direction from the circle point of the driven crank to the circle point of the driving crank
				glm::dvec2 v3 = X1 - X2;

				// calculate its sign
				if (i == 0) {
					sign1 = (v1.x * v3.y - v1.y * v3.x >= 0) ? 1 : -1;
					sign2 = (v2.x * v3.y - v2.y * v3.x >= 0) ? 1 : -1;
				}
				else {
					if ((v1.x * v3.y - v1.y * v3.x >= 0 ? 1 : -1) != sign1) sign1 = 9999;
					if ((v2.x * v3.y - v2.y * v3.x >= 0 ? 1 : -1) != sign2) sign2 = 9999;
					if (sign1 == 9999 && sign2 == 9999) return true;
				}
			}

			return false;
		}
	}

}