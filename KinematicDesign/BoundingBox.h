#pragma once

#include <glm/glm.hpp>

namespace canvas {

	class BoundingBox {
	public:
		glm::dvec2 minPt;
		glm::dvec2 maxPt;

	public:
		BoundingBox(const glm::dvec2& minPt, const glm::dvec2& maxPt);
		~BoundingBox();

		glm::dvec2 center() const;
		double width() const;
		double height() const;
	};

}