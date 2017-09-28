#pragma once

#include <vector>
#include <boost/shared_ptr.hpp>
#include <QMap>

#include "Joint.h"
#include "Link.h"
#include "BodyGeometry.h"
#include "Vertex.h"

namespace kinematics {

	const float link_radius = 0.5f;
	const float link_depth = 0.3f;
	const float joint_radius = 0.25f;
	const float joint_cap_radius1 = 0.23f;
	const float joint_cap_radius2 = 0.28f;
	const float joint_depth = 0.15f;
	const float hole_radius = 0.26f;
	const float slider_bar_with = 0.6f;
	const float slider_with = 1.0f;
	const float slider_bar_depth = 0.3f;
	const float slider_depth = 0.5f;
	/*
	const float link_radius = 0.7f;
	const float link_depth = 0.6f;
	const float joint_radius = 0.35f;
	const float joint_cap_radius1 = 0.322f;
	const float joint_cap_radius2 = 0.392f;
	const float joint_depth = 0.3f;
	const float hole_radius = 0.364f;
	const float slider_bar_with = 1.2f;
	const float slider_with = 2.0f;
	const float slider_bar_depth = 0.6f;
	const float slider_depth = 1.0f;
	*/
	class KinematicDiagram {
	public:
		QMap<int, boost::shared_ptr<Joint>> joints;
		QMap<int, boost::shared_ptr<Link>> links;
		std::vector<boost::shared_ptr<BodyGeometry>> bodies;

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
		void connectJointsToBodies(std::vector<Object25D>& fixed_body_pts);
		void load(const QString& filename);
		void save(const QString& filename);
		void updateBodyAdjacency();
		bool isCollided() const;
		void draw(QPainter& painter, const QPointF& origin, float scale, bool show_bodies, bool show_links) const;
	};

}