#pragma once

#include "Joint.h"
#include <QDomDocument>

namespace kinematics {

	class Gear : public Joint {
	public:
		glm::dvec2 center;
		double radius;
		double speed;
		double phase;
		double prev_phase;

	public:
		Gear(int id, bool ground, const glm::dvec2& pos, double radius, double speed, double phase);
		Gear(QDomElement& node);

		void draw(QPainter& painter, const QPointF& origin, float scale);
		void stepForward(double step_size);
		bool forwardKinematics();
	};

}