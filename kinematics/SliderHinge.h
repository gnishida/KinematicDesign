#pragma once

#include "Joint.h"
#include <QDomDocument>

namespace kinematics {

	class SliderHinge : public Joint {
	private:
		double theta;

	public:
		SliderHinge(int id, const glm::dvec2& pos);
		SliderHinge(QDomElement& node);

		void saveState();
		void restoreState();
		void draw(QPainter& painter);
		void stepForward(double step_size);
		bool forwardKinematics();
	};

}