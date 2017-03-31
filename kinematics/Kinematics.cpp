#include "Kinematics.h"
#include <iostream>
#include <QFile>
#include <QDomDocument>
#include <QTextStream>
#include <QDate>
#include "PinJoint.h"
#include "SliderHinge.h"
#include "Gear.h"
#include "Utils.h"

namespace kinematics {
	Kinematics::Kinematics() {
		show_assemblies = true;
		show_links = true;
		show_bodies = true;
	}

	void Kinematics::load(const QString& filename) {
		trace_end_effector.clear();

		diagram.load(filename);

		//trace_end_effector.resize(assemblies.size());
	}

	void Kinematics::save(const QString& filename) {
		diagram.save(filename);
	}

	void Kinematics::forwardKinematics() {
		std::list<boost::shared_ptr<Joint>> queue;

		// put the joints whose position has not been determined into the queue
		for (auto it = diagram.joints.begin(); it != diagram.joints.end(); ++it) {
			if (!diagram.joints[it.key()]->determined) queue.push_back(diagram.joints[it.key()]);
		}

		int count = 0;
		while (!queue.empty()) {
			boost::shared_ptr<Joint> joint = queue.front();
			queue.pop_front();

			if (!joint->forwardKinematics()) {
				queue.push_back(joint);
			}

			count++;
			if (count > 1000) {
				throw "infinite loop is detected.";
			}
		}

		if (isCollided()) {
			//throw "collision is detected.";
		}
	}

	void Kinematics::stepForward(double time_step) {
		// save the current state
		KinematicDiagram prev_state = diagram.clone();

		// clear the determined flag of joints
		for (auto it = diagram.joints.begin(); it != diagram.joints.end(); ++it) {
			if (diagram.joints[it.key()]->ground) {
				diagram.joints[it.key()]->determined = true;
			}
			else {
				diagram.joints[it.key()]->determined = false;
			}
		}

		// update the positions of the joints by the driver
		bool driver_exist = false;
		for (auto it = diagram.joints.begin(); it != diagram.joints.end(); ++it) {
			if (diagram.joints[it.key()]->ground) {
				driver_exist = true;
				diagram.joints[it.key()]->stepForward(time_step);
			}
		}

		if (driver_exist) {
			try {
				forwardKinematics();
			}
			catch (char* ex) {
				int a = 0;
				diagram = prev_state.clone();
				throw ex;
			}
		}
	}

	bool Kinematics::isCollided() {
		return diagram.isCollided();
	}

	void Kinematics::draw(QPainter& painter) const {
		diagram.draw(painter, show_bodies, show_links);
	}

	void Kinematics::showAssemblies(bool flag) {
		show_assemblies = flag;
	}

	void Kinematics::showLinks(bool flag) {
		show_links = flag;
	}

	void Kinematics::showBodies(bool flag) {
		show_bodies = flag;
	}

}
