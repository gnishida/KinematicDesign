#include "BoundingBox.h"

namespace canvas {

	BoundingBox::BoundingBox(const glm::dvec2& minPt, const glm::dvec2& maxPt) {
		this->minPt = minPt;
		this->maxPt = maxPt;
	}
	
	BoundingBox::~BoundingBox() {
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