#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <QMap>

namespace kinematics {

	class Joint;
	class BodyGeometry;

	class JointConnector {
	public:
		int type;	// 0 - connector to fixed body / 1 - connector to moving body / 2 - link
		std::vector<boost::shared_ptr<Joint>> joints;
		glm::dvec2 closest_pt;	// the point on the body that is closest to the joint
		boost::shared_ptr<BodyGeometry> body;	// the moving body to which the connector is attached (for the fixed body, we do not need this because closest pt does not move)
		QMap<int, bool> collisions1;	// store all the indices of connectors that this connector collide
		QMap<int, std::vector<int>> collisions2;	// store all the indices of connectors that this connector collide
													// key of map is the joint id, and value is the list of connect ids
		QMap<int, bool> neighbors;	// store all the indices of neighbor connectors

	public:
		JointConnector(boost::shared_ptr<Joint> joint, const glm::dvec2& closest_pt);
		JointConnector(boost::shared_ptr<Joint> joint, const glm::dvec2& closest_pt, boost::shared_ptr<BodyGeometry> body);
		JointConnector(const std::vector<boost::shared_ptr<Joint>>& joints);

		bool hasJoint(const boost::shared_ptr<Joint>& joint) const;
	};

}