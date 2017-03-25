#pragma once

#include <QPainter>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <boost/shared_ptr.hpp>
#include <QMap>
#include "Joint.h"
#include "Link.h"
#include "BodyGeometry.h"

namespace kinematics {
	
	class Kinematics {
	public:
		QMap<int, boost::shared_ptr<Joint>> joints;
		std::vector<boost::shared_ptr<Link>> links;
		std::vector<boost::shared_ptr<BodyGeometry>> bodies;
		std::vector<std::vector<glm::vec2>> trace_end_effector;

		bool show_assemblies;
		bool show_links;
		bool show_bodies;

	public:
		Kinematics();

		void load(const QString& filename);
		void save(const QString& filename);
		void saveState();
		void restoreState();
		void forwardKinematics();
		void stepForward(double time_step);
		void updateBodyAdjacency();
		bool isCollided();
		void draw(QPainter& painter);
		void showAssemblies(bool flag);
		void showLinks(bool flag);
		void showBodies(bool flag);
	};

}
