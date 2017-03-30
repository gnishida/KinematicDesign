#pragma once

#include "Joint.h"
#include <QDomDocument>

namespace kinematics {

	class SliderHinge : public Joint {
	private:
		double theta;

	public:
		SliderHinge(int id, bool ground, const glm::dvec2& pos);
		SliderHinge(const glm::dvec2& pos);
		SliderHinge(QDomElement& node);

		void draw(QPainter& painter);
		void stepForward(double step_size);
		bool forwardKinematics();
	};

}