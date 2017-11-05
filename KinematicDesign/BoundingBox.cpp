#include "BoundingBox.h"
#include <algorithm>

namespace canvas {

	BoundingBox::BoundingBox() {
		minPt.x = std::numeric_limits<double>::max();
		minPt.y = std::numeric_limits<double>::max();
		maxPt.x = -std::numeric_limits<double>::max();
		maxPt.y = -std::numeric_limits<double>::max();
	}

	BoundingBox::BoundingBox(const glm::dvec2& minPt, const glm::dvec2& maxPt) {
		this->minPt = minPt;
		this->maxPt = maxPt;
	}
	
	BoundingBox::~BoundingBox() {
	}

	void BoundingBox::addPoint(const glm::dvec2& pt) {
		minPt.x = std::min(minPt.x, pt.x);
		minPt.y = std::min(minPt.y, pt.y);
		maxPt.x = std::max(maxPt.x, pt.x);
		maxPt.y = std::max(maxPt.y, pt.y);
	}

	void BoundingBox::addPoints(const std::vector<glm::dvec2>& points) {
		for (int i = 0; i < points.size(); i++) {
			addPoint(points[i]);
		}
	}

	glm::dvec2 BoundingBox::center() const {
		return glm::dvec2((minPt.x + maxPt.x) * 0.5, (minPt.y + maxPt.y) * 0.5);
	}

	double BoundingBox::width() const {
		return maxPt.x - minPt.x;
	}

	double BoundingBox::height() const {
		return maxPt.y - minPt.y;
	}

}