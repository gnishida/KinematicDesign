#include "PinJoint.h"
#include "Link.h"
#include "Utils.h"

namespace kinematics {

	PinJoint::PinJoint(const glm::dvec2& pos) : Joint() {
		this->id = -1;
		this->type = TYPE_PIN;
		this->ground = false;
		this->pos = pos;
	}

	PinJoint::PinJoint(bool ground, const glm::dvec2& pos) : Joint() {
		this->id = -1;
		this->type = TYPE_PIN;
		this->ground = ground;
		this->pos = pos;
	}

	PinJoint::PinJoint(int id, bool ground, const glm::dvec2& pos) : Joint() {
		this->id = id;
		this->type = TYPE_PIN;
		this->ground = ground;
		this->pos = pos;
	}

	PinJoint::PinJoint(QDomElement& node) : Joint() {
		id = node.attribute("id").toInt();
		this->type = TYPE_PIN;
		this->ground = node.attribute("ground").toLower() == "true";
		pos.x = node.attribute("x").toDouble();
		pos.y = node.attribute("y").toDouble();
	}

	void PinJoint::draw(QPainter& painter) {
		painter.save();
		painter.setPen(QPen(QColor(0, 0, 0), 1));
		painter.setBrush(QBrush(QColor(255, 255, 255)));
		painter.drawEllipse(QPoint(pos.x, 800 - pos.y), 5, 5);
		painter.restore();
	}

	void PinJoint::stepForward(double step_size) {
		if (ground) {
			for (int i = 0; i < links.size(); ++i) {
				if (links[i]->driver) {
					links[i]->rotate(pos, step_size);
				}
			}
			determined = true;
		}
	}

	/**
	 * Update the position of this joint.
	 * Return true if the position is updated.
	 * Return false if one of the positions of the parent nodes has not been updated yet.
	 */
	bool PinJoint::forwardKinematics() {
		if (links.size() == 0) {
			determined = true;
			return true;
		}

		// If one of the links has its position already determined,
		// then use it to determine the position of this joint.
		for (int i = 0; i < links.size(); ++i) {
			if (links[i]->isDetermined()) {
				// calculate the position of this joint based on the joints whose position has already been determined.
				pos = links[i]->transformByDeterminedJoints(id);
				determined = true;
				return true;
			}
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
				}
			}
		}
		if (positions.size() == 2) {
			pos = circleCircleIntersection(positions[0], lengths[0], positions[1], lengths[1], pos);
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
	}

}