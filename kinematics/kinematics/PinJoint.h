#pragma once

#include "Joint.h"
#include <QDomDocument>

namespace kinematics {

	class PinJoint : public Joint {
	public:
		PinJoint(int id, bool ground, const glm::dvec2& pos);
		PinJoint(QDomElement& node);

		void draw(QPainter& painter, const QPointF& origin, float scale);
		void stepForward(double step_size);
		bool forwardKinematics();
	};

}
