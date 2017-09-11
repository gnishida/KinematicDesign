#include "SliderHinge.h"
#include "Link.h"
#include "KinematicUtils.h"

namespace kinematics {

	SliderHinge::SliderHinge(int id, bool ground, const glm::dvec2& pos) : Joint() {
		this->id = id;
		this->type = TYPE_SLIDER_HINGE;
		this->ground = ground;
		this->pos = pos;
	}

	SliderHinge::SliderHinge(QDomElement& node) : Joint() {
		id = node.attribute("id").toInt();
		type = TYPE_SLIDER_HINGE;
		this->ground = node.attribute("ground").toLower() == "true";
		pos.x = node.attribute("x").toDouble();
		pos.y = node.attribute("y").toDouble();
	}

	void SliderHinge::draw(QPainter& painter, const QPointF& origin, float scale) {
		painter.save();
		painter.setPen(QPen(QColor(0, 0, 0), 1));
		painter.setBrush(QBrush(QColor(255, 255, 255)));

		double theta = 0.0;
		if (links.size() > 0) {
			for (int i = 0; i < links[0]->joints.size(); ++i) {
				if (links[0]->joints[i]->id != id && links[0]->joints[i]->ground) {
					theta = atan2(links[0]->joints[i]->pos.y - pos.y, links[0]->joints[i]->pos.x - pos.x);
				}
			}
		}

		painter.translate(origin.x() + pos.x * scale, origin.y() - pos.y * scale);
		painter.rotate(-theta / M_PI * 180);
		painter.drawRect(-20, -5, 40, 10);

		painter.drawEllipse(QPoint(0, 0), 5, 5);
		painter.restore();
	}

	void SliderHinge::stepForward(double step_size) {
	}

	/**
	* Update the position of this joint.
	* Return true if the position is updated.
	* Return false if one of the positions of the parent nodes has not been updated yet.
	*/
	bool SliderHinge::forwardKinematics() {
		if (links.size() == 0) {
			determined = true;
			return true;
		}

		// If two of the links have at least one joint with its position determined,
		// then use them to determine the position of this joint.
		std::vector<glm::dvec2> positions;
		std::vector<double> lengths;
		for (int i = 0; i < links.size(); ++i) {
			for (int j = 0; j < links[i]->joints.size(); ++j) {
				if (links[i]->joints[j]->determined) {
					positions.push_back(links[i]->joints[j]->pos);
					lengths.push_back(links[i]->getLength(links[i]->joints[j]->id, id));
					break;
				}
			}
		}
		if (positions.size() == 2) {
			pos = circleLineIntersection(positions[1], lengths[1], positions[0], pos, pos);
			determined = true;
			return true;
		}
		else if (positions.size() < 2) {
			return false;
		}
		else if (positions.size() > 2) {
			throw "Over constrained";
		}

		// Otherwise, postpone updating the position later.
		return false;


		/*
		if (in_links.size() == 0) return true;

		if (in_links.size() > 2) throw "forward kinematics error. Overconstrained.";

		// if the parent points are not updated, postpone updating this point
		boost::shared_ptr<Link> l1 = in_links[0];
		if (!updated[l1->start]) {
			return false;
		}
		if (in_links.size() == 2) {
			boost::shared_ptr<Link> l2 = in_links[1];
			if (!updated[l2->start]) {
				return false;
			}

			// update this point based on two adjacent points
			glm::dvec2 p2 = joints[l1->start]->pos + glm::dvec2(cos(theta), sin(theta));
			pos = circleLineIntersection(joints[l2->start]->pos, l2->length, joints[l1->start]->pos, p2);
		}
		else {
			// pin joint
			pos = l1->forwardKinematics(joints[l1->start]->pos);
		}
		*/
		return false;
	}

}