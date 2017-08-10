#pragma once

#include <vector>
#include <map>
#include <glm/glm.hpp>

namespace kinematics {

	class SpecialPoint {
	public:
		static enum { TYPE_Q = 0, TYPE_UT, TYPE_B };
	public:
		int index;
		int type;
		std::pair<int, int> subscript;

	public:
		SpecialPoint(int index, int type, const std::pair<int, int>& subscript) : index(index), type(type), subscript(subscript) {}
	};

	class Curve {
	public:
		bool closed;
		std::vector<glm::dvec2> points;

	public:
		Curve() : closed(false) {}
	};

	class SolutionSet {
	public:
		std::vector<Curve> curves;

	public:
		SolutionSet() {}
	};


	void calculateSolutionCurve(const std::vector<glm::dmat4x4>& poses, std::vector<std::vector<std::vector<glm::dvec2>>>& solutions);
	std::vector<std::vector<std::vector<glm::dvec2>>> calculateCenterPointCurve(const glm::dvec2& P12, const glm::dvec2& P13, const glm::dvec2& P14, const glm::dvec2& P23, const glm::dvec2& P24, double theta12, double theta13);
	glm::dvec2 calculateCirclePointFromCenterPoint(const glm::dvec2& C, const glm::dvec2& P12, const glm::dvec2& P13, double theta12, double theta13);

	std::vector<std::vector<std::vector<glm::dvec2>>> calculatePoles(const std::vector<glm::dmat4x4>& poses);
	std::vector<std::vector<std::vector<SpecialPoint>>> calculatePoleIntersections(const std::vector<glm::dmat4x4>& poses, const std::vector<std::vector<std::vector<glm::dvec2>>>& curves);

	std::vector<std::vector<SpecialPoint>> calculateUTs(const std::vector<std::vector<glm::dvec2>>& curve, const std::vector<std::vector<std::vector<glm::dvec2>>>& P);
	void calculateUT(const std::vector<glm::dvec2>& curve, const glm::dvec2& P1, const glm::dvec2& P2, const std::pair<int, int>& subscript, std::vector<SpecialPoint>& ret);

	std::vector<std::vector<std::tuple<int, int, int>>> findExtremePoses(const std::vector<glm::dmat4x4>& poses, const std::vector<std::vector<glm::dvec2>>& curve, std::vector<std::vector<glm::dvec2>>& P, std::vector<std::vector<SpecialPoint>>& Q, std::vector<std::vector<SpecialPoint>>& UT);
	std::pair<int, int> findExtremePoses(const std::vector<std::vector<bool>>& table);
	std::pair<int, int> findSolution(const std::vector<std::vector<glm::dvec2>>& curves, const glm::dvec2& pt);
	int findSolution(const std::vector<glm::dvec2>& curve, const glm::dvec2& pt);

	std::vector<std::vector<glm::dvec2>> findValidSolution(const std::vector<glm::dmat4x4>& poses, const std::vector<std::vector<std::vector<glm::dvec2>>>& curves);
	int getGrashofType(const glm::dvec2& C1, const glm::dvec2& C2, const glm::dvec2& X1, const glm::dvec2& X2);
	bool checkGrashofDefect(const glm::dvec2& C1, const glm::dvec2& C2, const glm::dvec2& X1, const glm::dvec2& X2);
	bool checkOrderDefect(const std::vector<glm::dmat4x4>& poses, const glm::dvec2& C1, const glm::dvec2& C2, const glm::dvec2& X1, const glm::dvec2& X2);
	bool checkBranchDefect(const std::vector<glm::dmat4x4>& poses, const glm::dvec2& C1, const glm::dvec2& C2, const glm::dvec2& X1, const glm::dvec2& X2);

}