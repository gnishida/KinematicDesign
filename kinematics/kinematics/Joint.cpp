#include "Joint.h"
#include "Link.h"

namespace kinematics {

	Joint::Joint() {
		ground = false;
		determined = false;
	}

	void Joint::rotate(const glm::dvec2& rotation_center, double angle) {
		double x = pos.x;
		double y = pos.y;
		pos.x = cos(angle) * (x - rotation_center.x) - sin(angle) * (y - rotation_center.y) + rotation_center.x;
		pos.y = sin(angle) * (x - rotation_center.x) + cos(angle) * (y - rotation_center.y) + rotation_center.y;

		determined = true;
	}

}