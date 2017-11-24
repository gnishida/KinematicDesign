#pragma once

#include <vector>
#include <boost/shared_ptr.hpp>
#include <QMap>

#include "Joint.h"
#include "Link.h"
#include "BodyGeometry.h"
#include "JointConnector.h"
#include "Vertex.h"

namespace kinematics {

	class Options {
	public:
		static Options* instance;
		float body_margin;
		float gap;
		float link_width;
		float link_depth;
		float hole_radius;
		float joint_radius;
		float joint_cap_radius1;
		float joint_cap_radius2;
		float joint_cap_depth;
		float slider_guide_width;
		float slider_guide_depth;
		float body_depth;

	protected:
		Options() {
			body_margin = 0.3f;
			gap = 0.04f;
			link_width = 1.0f;
			link_depth = 0.3f;
			hole_radius = 0.26f;
			joint_radius = 0.25f;
			joint_cap_radius1 = 0.23f;
			joint_cap_radius2 = 0.28f;
			joint_cap_depth = 0.15f;
			slider_guide_width = 1.0f;
			slider_guide_depth = 0.3f;
			body_depth = 10.0f;
		}

	public:
		static Options* getInstance() {
			if (!instance) instance = new Options();
			return instance;
		}
	};
	
	static Options* options = Options::getInstance();

	class KinematicDiagram {
	public:
		QMap<int, boost::shared_ptr<Joint>> joints;
		QMap<int, boost::shared_ptr<Link>> links;
		std::vector<boost::shared_ptr<BodyGeometry>> bodies;
		std::vector<JointConnector> connectors;

	public:
		KinematicDiagram();
		~KinematicDiagram();

		KinematicDiagram clone() const;
		void clear();
		void initialize();
		void addJoint(boost::shared_ptr<Joint> joint);
		void setJointToLink(boost::shared_ptr<Joint> joint, boost::shared_ptr<Link> link);
		boost::shared_ptr<Link> newLink();
		boost::shared_ptr<Link> newLink(bool driver, bool actual_link = true, double z = 0);
		boost::shared_ptr<Link> addLink(boost::shared_ptr<Joint> joint1, boost::shared_ptr<Joint> joint2, bool actual_link = true, double z = 0);
		boost::shared_ptr<Link> addLink(bool driver, boost::shared_ptr<Joint> joint1, boost::shared_ptr<Joint> joint2, bool actual_link = true, double z = 0);
		boost::shared_ptr<Link> addLink(std::vector<boost::shared_ptr<Joint>> joints, bool actual_link = true, double z = 0);
		boost::shared_ptr<Link> addLink(bool driver, std::vector<boost::shared_ptr<Joint>> joints, bool actual_link = true, double z = 0);
		void addBody(boost::shared_ptr<Joint> joint1, boost::shared_ptr<Joint> joint2, const Object25D& polygons);
		void addPolygonToBody(int body_id, const Polygon25D& polygon);
		void connectJointsToBodies(std::vector<Object25D>& fixed_bodies, const std::vector<std::vector<int>>& zorder, std::vector<glm::dvec2>& connected_pts);
		glm::dvec2 connectFixedJointToBody(boost::shared_ptr<kinematics::Joint> joint, std::vector<Object25D>& fixed_bodies, double link_z);
		glm::dvec2 connectMovingJointToBody(boost::shared_ptr<Joint> joint, int body_id, const std::vector<glm::dvec2>& moving_body, double link_z);
		void load(const QString& filename);
		void save(const QString& filename);
		void updateBodyAdjacency();
		bool isCollided(bool main_body_only) const;
		void recordCollisionForConnectors();
		void draw(QPainter& painter, const QPointF& origin, float scale, bool show_bodies, bool show_links) const;
	};

}