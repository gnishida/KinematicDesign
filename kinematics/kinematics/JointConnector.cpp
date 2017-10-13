#include "JointConnector.h"
#include "Joint.h"
#include "BodyGeometry.h"

namespace kinematics {

	JointConnector::JointConnector(boost::shared_ptr<Joint> joint, const glm::dvec2& closest_pt) {
		type = 0;
		this->closest_pt = closest_pt;
		joints.push_back(joint);
	}

	JointConnector::JointConnector(boost::shared_ptr<Joint> joint, const glm::dvec2& closest_pt, boost::shared_ptr<BodyGeometry> body) {
		type = 1;
		this->closest_pt = closest_pt;
		this->body = body;
		joints.push_back(joint);
	}

	JointConnector::JointConnector(const  std::vector<boost::shared_ptr<Joint>>& joints) {
		type = 2;
		this->joints = joints;
	}
	
}