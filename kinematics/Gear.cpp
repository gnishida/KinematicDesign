#include "Gear.h"
#include "Link.h"
#include "Utils.h"

namespace kinematics {

	Gear::Gear(int id, bool ground, const glm::dvec2& center, double radius, double speed, double phase) : Joint() {
		this->id = id;
		this->type = TYPE_GEAR;
		this->ground = ground;
		this->center = center;
		this->radius = radius;
		this->speed = speed;
		this->phase = phase;
		this->pos = center + glm::dvec2(cos(phase), sin(phase)) * radius;
	}

	Gear::Gear(const glm::dvec2& pos, double radius, double speed, double phase) {
		this->id = -1;
		this->type = TYPE_GEAR;
		this->ground = false;
		this->center = center;
		this->radius = radius;
		this->speed = speed;
		this->phase = phase;
		this->pos = center + glm::dvec2(cos(phase), sin(phase)) * radius;
	}

	Gear::Gear(QDomElement& node) : Joint() {
		id = node.attribute("id").toInt();
		type = TYPE_GEAR;
		this->ground = node.attribute("ground").toLower() == "true";
		center.x = node.attribute("x").toDouble();
		center.y = node.attribute("y").toDouble();
		radius = node.attribute("radius").toDouble();
		speed = node.attribute("speed").toDouble();
		phase = node.attribute("phase").toDouble();
		pos = center + glm::dvec2(cos(phase), sin(phase)) * radius;
	}

	void Gear::draw(QPainter& painter) {
		painter.save();
		painter.setPen(QPen(QColor(0, 0, 0), 1));
		painter.setBrush(QBrush(QColor(255, 255, 255, 0)));
		painter.drawEllipse(QPoint(center.x, 800 - center.y), 5, 5);

		painter.setBrush(QBrush(QColor(255, 255, 255)));

		int num_split = radius * 0.6;
		for (int i = 0; i < num_split; ++i) {
			float theta1 = i * M_PI * 2.0 / num_split;
			float theta2 = (i + 0.5) * M_PI * 2.0 / num_split;
			float theta3 = (i + 1) * M_PI * 2.0 / num_split;

			glm::vec2 p1 = center + glm::dvec2(cos(phase + theta1), sin(phase + theta1)) * (radius + 4);
			glm::vec2 p2 = center + glm::dvec2(cos(phase + theta2), sin(phase + theta2)) * (radius + 4);
			glm::vec2 p3 = center + glm::dvec2(cos(phase + theta2), sin(phase + theta2)) * radius;
			glm::vec2 p4 = center + glm::dvec2(cos(phase + theta3), sin(phase + theta3)) * radius;
			glm::vec2 p5 = center + glm::dvec2(cos(phase + theta3), sin(phase + theta3)) * (radius + 4);

			painter.drawLine(p1.x, 800 - p1.y, p2.x, 800 - p2.y);
			painter.drawLine(p2.x, 800 - p2.y, p3.x, 800 - p3.y);
			painter.drawLine(p3.x, 800 - p3.y, p4.x, 800 - p4.y);
			painter.drawLine(p4.x, 800 - p4.y, p5.x, 800 - p5.y);
		}
		
		painter.drawEllipse(QPoint(pos.x, 800 - pos.y), 5, 5);
				
		painter.restore();
	}

	void Gear::stepForward(double step_size) {
		phase += speed * step_size;
		pos = center + glm::dvec2(cos(phase), sin(phase)) * radius;
		determined = true;
	}

	/**
	* Update the position of this joint.
	* Return true if the position is updated.
	* Return false if one of the positions of the parent nodes has not been updated yet.
	*/
	bool Gear::forwardKinematics() {
		pos = center + glm::dvec2(cos(phase), sin(phase)) * radius;
		determined = true;
		return true;
	}

}