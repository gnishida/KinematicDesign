#include "Kinematics.h"
#include <iostream>
#include <QFile>
#include <QDomDocument>
#include <QTextStream>
#include <QDate>
#include "PinJoint.h"
#include "SliderHinge.h"
#include "Gear.h"
#include "KinematicUtils.h"

namespace kinematics {

	Kinematics::Kinematics(double simulation_speed) {
		this->simulation_speed = simulation_speed;
		show_links = true;
		show_bodies = true;
	}

	void Kinematics::clear() {
		diagram.clear();
	}

	void Kinematics::load(const QString& filename) {
		diagram.load(filename);
	}

	void Kinematics::save(const QString& filename) {
		diagram.save(filename);
	}

	/**
	 * Step forward the linkage.
	 *
	 * @param collision_check	0 - no collision check
	 *                          1 - check collision and throw exception if collision occurs
	 *                          2 - check collision only for the main bodies and throw exception if collision occurs
	 *                          3 - only record the collision between connectors
	 *                          
	 */
	void Kinematics::forwardKinematics(int collision_check) {
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

		if (collision_check == 1 || collision_check == 2) {
			if (diagram.isCollided(collision_check == 2)) throw "collision is detected.";
		}
		else if (collision_check == 3) {
			diagram.recordCollisionForConnectors();
		}
	}

	/**
	* Step forward the linkage.
	*
	* @param collision_check	0 - no collision check
	*                           1 - check collision and throw exception if collision occurs
	*                           2 - check collision only for the main bodies and throw exception if collision occurs
	*                           3 - only record the collision between connectors
	* @param need_recovery_for_collision
	*                           true - when collision occurs, recover the state right before the collision
	*/
	void Kinematics::stepForward(int collision_check, bool need_recovery_for_collision) {
		// save the current state
		KinematicDiagram prev_state;
		if (need_recovery_for_collision) {
			prev_state = diagram.clone();
		}

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
				diagram.joints[it.key()]->stepForward(simulation_speed);
			}
		}

		if (driver_exist) {
			try {
				forwardKinematics(collision_check);
			}
			catch (char* ex) {
				if (need_recovery_for_collision) {
					diagram = prev_state.clone();
				}
				throw ex;
			}
		}
	}

	void Kinematics::stepBackward(int collision_check, bool need_recovery_for_collision) {
		// save the current state
		KinematicDiagram prev_state;
		if (need_recovery_for_collision) {
			prev_state = diagram.clone();
		}

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
				diagram.joints[it.key()]->stepForward(-simulation_speed);
			}
		}

		if (driver_exist) {
			try {
				forwardKinematics(collision_check);
			}
			catch (char* ex) {
				if (need_recovery_for_collision) {
					diagram = prev_state.clone();
				}
				throw ex;
			}
		}
	}

	void Kinematics::draw(QPainter& painter, const QPointF& origin, float scale) const {
		diagram.draw(painter, origin, scale, show_bodies, show_links);
	}

	void Kinematics::speedUp() {
		simulation_speed *= 2.0;
	}

	void Kinematics::speedDown() {
		simulation_speed *= 0.5;
	}

	void Kinematics::invertSpeed() {
		simulation_speed = -simulation_speed;
	}

	void Kinematics::showLinks(bool flag) {
		show_links = flag;
	}

	void Kinematics::showBodies(bool flag) {
		show_bodies = flag;
	}

}
