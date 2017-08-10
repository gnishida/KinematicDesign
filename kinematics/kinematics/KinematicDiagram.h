#pragma once

#include <vector>
#include <boost/shared_ptr.hpp>
#include <QMap>

#include "Joint.h"
#include "Link.h"
#include "BodyGeometry.h"

namespace kinematics {

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
		boost::shared_ptr<Link> newLink(bool driver);
		boost::shared_ptr<Link> addLink(boost::shared_ptr<Joint> joint1, boost::shared_ptr<Joint> joint2);
		boost::shared_ptr<Link> addLink(bool driver, boost::shared_ptr<Joint> joint1, boost::shared_ptr<Joint> joint2);
		boost::shared_ptr<Link> addLink(std::vector<boost::shared_ptr<Joint>> joints);
		boost::shared_ptr<Link> addLink(bool driver, std::vector<boost::shared_ptr<Joint>> joints);
		void addBody(boost::shared_ptr<Joint> joint1, boost::shared_ptr<Joint> joint2, std::vector<glm::dvec2> points);
		void load(const QString& filename);
		void save(const QString& filename);
		void updateBodyAdjacency();
		bool isCollided() const;
		void draw(QPainter& painter, const QPointF& origin, float scale, bool show_bodies, bool show_links) const;
	};

}