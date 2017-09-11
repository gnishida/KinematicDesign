#include "PinJoint.h"
#include "Link.h"
#include "KinematicUtils.h"

namespace kinematics {

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

	void PinJoint::draw(QPainter& painter, const QPointF& origin, float scale) {
		painter.save();

		// if this is a fixed joint, draw a base
		if (ground) {
			painter.setPen(QPen(QColor(0, 0, 0), 1));
			painter.setBrush(QBrush(QColor(255, 255, 255, 0)));
			QPolygonF poly;
			for (int i = 0; i <= 10; i++) {
				double theta = M_PI * i / 10;
				poly.push_back(QPointF(origin.x() + pos.x * scale + 8 * cos(theta), origin.y() - pos.y * scale - 8 * sin(theta)));
			}
			poly.push_back(QPointF(origin.x() + pos.x * scale - 8, origin.y() - pos.y * scale + 15));
			poly.push_back(QPointF(origin.x() + pos.x * scale + 8, origin.y() - pos.y * scale + 15));
			painter.drawPolygon(poly);
		}

		painter.setPen(QPen(QColor(0, 0, 0), 1));
		painter.setBrush(QBrush(QColor(255, 255, 255)));
		painter.drawEllipse(QPoint(origin.x() + pos.x * scale, origin.y() - pos.y * scale), 5, 5);
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
		else if (positions.size() == 0) {
			return false;
		}
		else if (positions.size() == 1) {
			positions.clear();
			lengths.clear();

			// If one of the links have more than two joints whose other ends are determined
			// then use them to determine the position of this joint.

			int link1;
			int link2;
			std::vector<double> lengths2;
			std::vector<int> pts_indices;
			std::vector<glm::dvec2> prev_positions;

			for (int i = 0; i < links.size(); ++i) {
				for (int j = 0; j < links[i]->joints.size(); ++j) {
					if (links[i]->joints[j]->determined) {
						positions.push_back(links[i]->joints[j]->pos);
						lengths.push_back(links[i]->getLength(links[i]->joints[j]->id, id));
						link1 = i;
						i = links.size();
						break;
					}
				}
			}

			for (int i = 0; i < links.size(); ++i) {
				if (i == link1) continue;

				lengths2.clear();

				// find two determined joints that are ajacent to link, links[i].
				for (int j = 0; j < links[i]->joints.size(); j++) {
					if (links[i]->joints[j]->id == id) continue;

					// find a determined joint that is adjacent to joint, links[i]->joints[j].
					for (int k = 0; k < links[i]->joints[j]->links.size(); k++) {
						if (links[i]->joints[j]->links[k] == links[i]) continue;

						for (int l = 0; l < links[i]->joints[j]->links[k]->joints.size(); l++) {
							if (links[i]->joints[j]->links[k]->joints[l]->id == links[i]->joints[j]->id) continue;

							if (links[i]->joints[j]->links[k]->joints[l]->determined) {
								positions.push_back(links[i]->joints[j]->links[k]->joints[l]->pos);
								lengths.push_back(links[i]->joints[j]->links[k]->getLength(links[i]->joints[j]->id, links[i]->joints[j]->links[k]->joints[l]->id));
								lengths2.push_back(links[i]->getLength(links[i]->joints[j]->id, id));
								pts_indices.push_back(links[i]->joints[j]->id);
								prev_positions.push_back(links[i]->joints[j]->pos);

								// to exit the loop
								k = links[i]->joints[j]->links.size();
								break;
							}
						}
					}
				}

				if (lengths2.size() == 2) {
					link2 = i;
					break;
				}
			}

			if (lengths2.size() == 2) {
				lengths2.push_back(glm::length(links[link2]->original_shape[pts_indices[0]] - links[link2]->original_shape[pts_indices[1]]));

				pos = kinematics::threeLengths(positions[0], lengths[0], positions[1], lengths[1], positions[2], lengths[2], lengths2[0], lengths2[1], lengths2[2], pos, prev_positions[0], prev_positions[1]);
				determined = true;
				return true;
			}

			return false;
		}
		else if (positions.size() > 2) {
			throw "Over constrained";
		}

		// Otherwise, postpone updating the position later.
		return false;
	}

}