#pragma once

#include "Joint.h"
#include <QDomDocument>

namespace kinematics {

	class PinJoint : public Joint {
	public:
		PinJoint(int id, const glm::dvec2& pos);
		PinJoint(QDomElement& node);

		void saveState();
		void restoreState();
		void draw(QPainter& painter);
		void stepForward(double step_size);
		bool forwardKinematics();
	};

}
