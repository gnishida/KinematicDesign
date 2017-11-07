#pragma once

#include <glm/glm.hpp>

namespace kinematics {

	class BBox {
	public:
		glm::dvec2 minPt;
		glm::dvec2 maxPt;

	public:
		BBox();
		BBox(const glm::dvec2& minPt, const glm::dvec2& maxPt);
		~BBox();

		glm::dvec2 center() const;
		double width() const;
		double height() const;
	};

}