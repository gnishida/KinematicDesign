#pragma once

#include <glm/glm.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <QMap>
#include <QPainter>

namespace kinematics {

	class Joint;

	class Link {
	public:
		int id;
		std::vector<boost::shared_ptr<Joint>> joints;
		QMap<int, glm::dvec2> original_shape;
		double angle;
		bool driver;

	public:
		Link(int id);
		Link(int id, bool driver);

		bool isDetermined();
		bool isGrounded();
		void addJoint(boost::shared_ptr<Joint> joint);
		void rotate(const glm::dvec2& rotation_center, double angle);
		double getLength(int joint_id1, int joint_id2);
		glm::dmat3x2 getTransformMatrix();
		glm::dvec2 transformByDeterminedJoints(int joint_id);
		glm::dvec2 forwardKinematics(glm::dvec2& start_pos);
		void draw(QPainter& painter, const QPointF& origin, float scale);
	};

}