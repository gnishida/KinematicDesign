#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace canvas {

	class BoundingBox {
	public:
		glm::dvec2 minPt;
		glm::dvec2 maxPt;

	public:
		BoundingBox();
		BoundingBox(const glm::dvec2& minPt, const glm::dvec2& maxPt);
		~BoundingBox();

		void addPoint(const glm::dvec2& pt);
		void addPoints(const std::vector<glm::dvec2>& points);
		glm::dvec2 center() const;
		double width() const;
		double height() const;
	};

}