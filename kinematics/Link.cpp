#include "Link.h"
#include "Joint.h"
#include <QPolygon>

namespace kinematics {

	Link::Link(int id) {
		this->id = id;
		this->driver = false;
	}

	Link::Link(int id, bool driver) {
		this->id = id;
		this->driver = driver;
	}

	bool Link::isDetermined() {
		int count = 0;
		for (int i = 0; i < joints.size(); ++i) {
			if (joints[i]->determined) count++;
		}

		if (count >= 2) return true;
		else return false;
	}

	bool Link::isGrounded() {
		int count = 0;
		for (int i = 0; i < joints.size(); ++i) {
			if (joints[i]->ground) count++;
		}

		if (count >= 2) return true;
		else return false;
	}

	void Link::addJoint(boost::shared_ptr<Joint> joint) {
		joints.push_back(joint);
	}

	void Link::rotate(const glm::dvec2& rotation_center, double angle) {
		for (int i = 0; i < joints.size(); ++i) {
			joints[i]->rotate(rotation_center, angle);
		}
	}

	double Link::getLength(int joint_id1, int joint_id2) {
		return glm::length(original_shape[joint_id1] - original_shape[joint_id2]);
	}

	glm::dmat3x2 Link::getTransformMatrix() {
		std::vector<glm::dvec2> orig_p;
		std::vector<glm::dvec2> p;
		for (int i = 0; i < joints.size(); ++i) {
			if (joints[i]->determined) {
				p.push_back(joints[i]->pos);
				orig_p.push_back(original_shape[joints[i]->id]);
			}
		}

		if (p.size() < 2) throw "Undetermined";

		double num = (p[1].y - p[0].y) * (orig_p[1].x - orig_p[0].x) - (p[1].x - p[0].x) * (orig_p[1].y - orig_p[0].y);
		double den = (p[1].x - p[0].x) * (orig_p[1].x - orig_p[0].x) + (p[1].y - p[0].y) * (orig_p[1].y - orig_p[0].y);
		double theta = atan2(num, den);
		glm::dmat3x2 ret;
		ret[0][0] = cos(theta);
		ret[0][1] = sin(theta);
		ret[1][0] = -sin(theta);
		ret[1][1] = cos(theta);
		ret[2][0] = -cos(theta) * orig_p[0].x + sin(theta) * orig_p[0].y + p[0].x;
		ret[2][1] = -sin(theta) * orig_p[0].x - cos(theta) * orig_p[0].y + p[0].y;

		return ret;
	}

	glm::dvec2 Link::transformByDeterminedJoints(int joint_id) {
		glm::dmat3x2 mat = getTransformMatrix();

		return mat * glm::dvec3(original_shape[joint_id], 1);
	}

	void Link::draw(QPainter& painter) {
		painter.save();

		painter.setPen(QPen(QColor(0, 0, 0), 3));
		painter.setBrush(QBrush(QColor(192, 192, 192)));
		QPolygon polygon;
		for (int i = 0; i < joints.size(); ++i) {
			polygon.append(QPoint(joints[i]->pos.x, 800 - joints[i]->pos.y));
		}
		painter.drawPolygon(polygon);

		painter.restore();
	}

}