#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <QMap>
#include <QPainter>
#include <iostream>

namespace kinematics {

	class Link;

	class Joint {
	public:
		static enum { TYPE_PIN = 0, TYPE_SLIDER, TYPE_SLIDER_HINGE, TYPE_GEAR };
	public:
		int id;
		int type;
		bool driver;
		bool ground;
		glm::dvec2 pos;
		glm::dvec2 prev_pos;
		std::vector<boost::shared_ptr<Link>> links;
		bool determined;

	public:
		Joint();

		virtual void saveState() = 0;
		virtual void restoreState() = 0;
		void rotate(const glm::dvec2& rotation_center, double angle);
		virtual void draw(QPainter& painter) = 0;
		virtual void stepForward(double step_size) = 0;
		virtual bool forwardKinematics() = 0;
	};

}