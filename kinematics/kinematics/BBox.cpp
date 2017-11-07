#include "BBox.h"

namespace kinematics {

	BBox::BBox() {
	}

	BBox::BBox(const glm::dvec2& minPt, const glm::dvec2& maxPt) {
		this->minPt = minPt;
		this->maxPt = maxPt;
	}


	BBox::~BBox() {
	}

	glm::dvec2 BBox::center() const {
		return glm::dvec2((minPt.x + maxPt.x) * 0.5, (minPt.y + maxPt.y) * 0.5);
	}

	double BBox::width() const {
		return maxPt.x - minPt.x;
	}

	double BBox::height() const {
		return maxPt.y - minPt.y;
	}

}