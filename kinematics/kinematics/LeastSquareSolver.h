#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <dlib/optimization.h>

namespace kinematics {
	typedef dlib::matrix<double, 0, 1> column_vector;

	class SolverForLink {
	public:
		SolverForLink(const std::vector<glm::dmat3x3>& poses);
		double operator() (const column_vector& arg) const;

	private:
		std::vector<glm::dmat3x3> poses;
	};

	class SolverForSlider {
	public:
		SolverForSlider(const std::vector<glm::dmat3x3>& poses);
		double operator() (const column_vector& arg) const;

	private:
		std::vector<glm::dmat3x3> poses;
	};

	class SolverForWattI {
	public:
		SolverForWattI(const std::vector<std::vector<glm::dmat3x3>>& poses);
		double operator() (const column_vector& arg) const;

	private:
		std::vector<std::vector<glm::dmat3x3>> poses;
	};

}

