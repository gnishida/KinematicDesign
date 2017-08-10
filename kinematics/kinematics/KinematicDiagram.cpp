#include "KinematicDiagram.h"
#include "PinJoint.h"
#include "SliderHinge.h"
#include "Gear.h"
#include "KinematicUtils.h"
#include <QDomDocument>
#include <QFile>
#include <QTextStream>
#include <glm/gtc/matrix_transform.hpp>

namespace kinematics {

	KinematicDiagram::KinematicDiagram() {
	}


	KinematicDiagram::~KinematicDiagram() {
	}

	KinematicDiagram KinematicDiagram::clone() const {
		KinematicDiagram copied_diagram;

		// copy joints
		for (int i = 0; i < joints.size(); ++i) {
			if (joints[i]->type == Joint::TYPE_PIN) {
				boost::shared_ptr<Joint> joint = boost::shared_ptr<Joint>(new PinJoint(joints[i]->id, joints[i]->ground, joints[i]->pos));
				joint->determined = joints[i]->determined;
				copied_diagram.addJoint(joint);
			}
			else if (joints[i]->type == Joint::TYPE_SLIDER_HINGE) {
				boost::shared_ptr<Joint> joint = boost::shared_ptr<Joint>(new SliderHinge(joints[i]->id, joints[i]->ground, joints[i]->pos));
				joint->determined = joints[i]->determined;
				copied_diagram.addJoint(joint);
			}
			else if (joints[i]->type == Joint::TYPE_GEAR) {
				boost::shared_ptr<Gear> gear = boost::static_pointer_cast<Gear>(joints[i]);
				boost::shared_ptr<Joint> joint = boost::shared_ptr<Joint>(new Gear(gear->id, gear->ground, gear->pos, gear->radius, gear->speed, gear->phase));
				joint->determined = joints[i]->determined;
				copied_diagram.addJoint(joint);
			}
		}

		// copy links
		for (int i = 0; i < links.size(); ++i) {
			std::vector<boost::shared_ptr<Joint>> copied_joints;
			for (int j = 0; j < links[i]->joints.size(); ++j) {
				copied_joints.push_back(copied_diagram.joints[links[i]->joints[j]->id]);
			}

			copied_diagram.addLink(links[i]->driver, copied_joints);
		}

		// copy the original shape of the links
		for (int i = 0; i < links.size(); ++i) {
			copied_diagram.links[i]->original_shape = links[i]->original_shape;
			copied_diagram.links[i]->angle = links[i]->angle;
		}

		// copy bodis
		for (int i = 0; i < bodies.size(); ++i) {
			int id1 = bodies[i]->pivot1->id;
			int id2 = bodies[i]->pivot2->id;
			boost::shared_ptr<BodyGeometry> body = boost::shared_ptr<BodyGeometry>(new BodyGeometry(copied_diagram.joints[id1], copied_diagram.joints[id2]));
			
			for (int j = 0; j < bodies[i]->points.size(); ++j) {
				body->points.push_back(bodies[i]->points[j]);
			}

			for (auto it = bodies[i]->neighbors.begin(); it != bodies[i]->neighbors.end(); ++it) {
				body->neighbors[it.key()] = it.value();
			}

			copied_diagram.bodies.push_back(body);
		}

		return copied_diagram;
	}

	void KinematicDiagram::clear() {
		joints.clear(); 
		links.clear();
		bodies.clear();
	}

	void KinematicDiagram::initialize() {
		// save the initial shape
		// this information is used to obtain the length between joints
		for (int i = 0; i < links.size(); ++i) {
			for (int j = 0; j < links[i]->joints.size(); ++j) {
				links[i]->original_shape[links[i]->joints[j]->id] = links[i]->joints[j]->pos;
			}
			if (links[i]->joints.size() >= 2) {
				links[i]->angle = atan2(links[i]->joints[1]->pos.y - links[i]->joints[0]->pos.y, links[i]->joints[1]->pos.x - links[i]->joints[0]->pos.x);
			}
		}

		updateBodyAdjacency();
	}

	void KinematicDiagram::addJoint(boost::shared_ptr<Joint> joint) {
		int id = joint->id;

		// if id == -1, automatically set the appropriate id
		if (id == -1) {
			id = 0;
			if (!joints.empty()) {
				id = joints.lastKey() + 1;
			}
			joint->id = id;
		}

		joints[id] = joint;
	}

	void KinematicDiagram::setJointToLink(boost::shared_ptr<Joint> joint, boost::shared_ptr<Link> link) {
		link->addJoint(joint);
		joint->links.push_back(link);
	}

	boost::shared_ptr<Link> KinematicDiagram::newLink() {
		return newLink(false);
	}

	boost::shared_ptr<Link> KinematicDiagram::newLink(bool driver) {
		int id = 0;
		if (!links.empty()) {
			id = links.lastKey() + 1;
		}

		boost::shared_ptr<Link> link = boost::shared_ptr<Link>(new Link(id, driver));
		links[id] = link;
		return link;
	}

	boost::shared_ptr<Link> KinematicDiagram::addLink(boost::shared_ptr<Joint> joint1, boost::shared_ptr<Joint> joint2) {
		return addLink(false, joint1, joint2);
	}

	boost::shared_ptr<Link> KinematicDiagram::addLink(bool driver, boost::shared_ptr<Joint> joint1, boost::shared_ptr<Joint> joint2) {
		int id = 0;
		if (!links.empty()) {
			id = links.lastKey() + 1;
		}

		boost::shared_ptr<Link> link = boost::shared_ptr<Link>(new Link(id, driver));
		links[id] = link;

		link->addJoint(joint1);
		link->addJoint(joint2);
		joint1->links.push_back(link);
		joint2->links.push_back(link);

		return link;
	}

	boost::shared_ptr<Link> KinematicDiagram::addLink(std::vector<boost::shared_ptr<Joint>> joints) {
		return addLink(false, joints);
	}

	boost::shared_ptr<Link> KinematicDiagram::addLink(bool driver, std::vector<boost::shared_ptr<Joint>> joints) {
		int id = 0;
		if (!links.empty()) {
			id = links.lastKey() + 1;
		}

		boost::shared_ptr<Link> link = boost::shared_ptr<Link>(new Link(id, driver));
		links[id] = link;

		for (int i = 0; i < joints.size(); ++i) {
			link->addJoint(joints[i]);
			joints[i]->links.push_back(link);
		}

		return link;
	}

	void KinematicDiagram::addBody(boost::shared_ptr<Joint> joint1, boost::shared_ptr<Joint> joint2, std::vector<glm::dvec2> points) {
		boost::shared_ptr<BodyGeometry> body = boost::shared_ptr<BodyGeometry>(new BodyGeometry(joint1, joint2));

		// setup rotation matrix
		glm::vec2 dir = joint2->pos - joint1->pos;
		double angle = atan2(dir.y, dir.x);

		glm::dvec2 p1 = joint1->pos;
		glm::dmat4x4 model;
		model = glm::rotate(model, -angle, glm::dvec3(0, 0, 1));

		for (int i = 0; i < points.size(); ++i) {
			// convert the coordinates to the local coordinate system
			glm::dvec2 rotated_p = glm::dvec2(model * glm::dvec4(points[i].x - p1.x, points[i].y - p1.y, 0, 1));

			body->points.push_back(rotated_p);
		}

		bodies.push_back(body);

	}

	void KinematicDiagram::load(const QString& filename) {
		QFile file(filename);
		if (!file.open(QFile::ReadOnly | QFile::Text)) throw "File cannot open.";

		QDomDocument doc;
		doc.setContent(&file);

		QDomElement root = doc.documentElement();
		if (root.tagName() != "design")	throw "Invalid file format.";

		// clear the data
		clear();

		QDomNode node = root.firstChild();
		while (!node.isNull()) {
			if (node.toElement().tagName() == "joints") {
				QDomNode joint_node = node.firstChild();
				while (!joint_node.isNull()) {
					if (joint_node.toElement().tagName() == "joint") {
						// add a joint
						int id = joint_node.toElement().attribute("id").toInt();
						if (joint_node.toElement().attribute("type") == "pin") {
							addJoint(boost::shared_ptr<Joint>(new PinJoint(joint_node.toElement())));
						}
						else if (joint_node.toElement().attribute("type") == "slider_hinge") {
							addJoint(boost::shared_ptr<Joint>(new SliderHinge(joint_node.toElement())));
						}
						else if (joint_node.toElement().attribute("type") == "gear") {
							addJoint(boost::shared_ptr<Joint>(new Gear(joint_node.toElement())));
						}
					}

					joint_node = joint_node.nextSibling();
				}
			}
			else if (node.toElement().tagName() == "links") {
				QDomNode link_node = node.firstChild();
				while (!link_node.isNull()) {
					if (link_node.toElement().tagName() == "link") {
						// add a link
						bool driver = link_node.toElement().attribute("driver").toLower() == "true" ? true : false;
						std::vector<boost::shared_ptr<Joint>> jts;
						QStringList joint_list = link_node.toElement().attribute("joints").split(",");
						for (int i = 0; i < joint_list.size(); ++i) {
							jts.push_back(joints[joint_list[i].toInt()]);
						}
						addLink(driver, jts);
					}

					link_node = link_node.nextSibling();
				}
			}
			else if (node.toElement().tagName() == "bodies") {
				QDomNode body_node = node.firstChild();
				while (!body_node.isNull()) {
					if (body_node.toElement().tagName() == "body") {
						// add a body
						int id1 = body_node.toElement().attribute("id1").toInt();
						int id2 = body_node.toElement().attribute("id2").toInt();

						std::vector<glm::dvec2> points;
						QDomNode point_node = body_node.firstChild();
						while (!point_node.isNull()) {
							if (point_node.toElement().tagName() == "point") {
								double x = point_node.toElement().attribute("x").toDouble();
								double y = point_node.toElement().attribute("y").toDouble();
								points.push_back(glm::dvec2(x, y));
							}

							point_node = point_node.nextSibling();
						}

						addBody(joints[id1], joints[id2], points);
					}

					body_node = body_node.nextSibling();
				}
			}

			node = node.nextSibling();
		}

		// initialize the adancency between rigid bodies
		initialize();

		//trace_end_effector.resize(assemblies.size());
	}

	void KinematicDiagram::save(const QString& filename) {
		/*
		QFile file(filename);
		if (!file.open(QFile::WriteOnly)) throw "File cannot open.";

		QDomDocument doc;

		// set root node
		QDomElement root = doc.createElement("design");
		root.setAttribute("author", "Gen Nishida");
		root.setAttribute("version", "1.0");
		root.setAttribute("date", QDate::currentDate().toString("MM/dd/yyyy"));
		doc.appendChild(root);

		// write points
		QDomElement points_node = doc.createElement("points");
		root.appendChild(points_node);
		for (auto it = joints.begin(); it != joints.end(); ++it) {
		QDomElement point_node = doc.createElement("point");
		point_node.setAttribute("id", it.key());
		point_node.setAttribute("x", it.value()->pos.x);
		point_node.setAttribute("y", it.value()->pos.y);
		points_node.appendChild(point_node);
		}

		// write links
		QDomElement links_node = doc.createElement("links");
		root.appendChild(links_node);
		for (auto it = joints.begin(); it != joints.end(); ++it) {
		for (int j = 0; j < it.value()->in_links.size(); ++j) {
		QDomElement link_node = doc.createElement("link");
		link_node.setAttribute("order", j);
		link_node.setAttribute("start", it.value()->in_links[j]->start);
		link_node.setAttribute("end", it.value()->in_links[j]->end);
		links_node.appendChild(link_node);
		}
		}

		// write bodies
		QDomElement bodies_node = doc.createElement("bodies");
		root.appendChild(bodies_node);
		for (int i = 0; i < bodies.size(); ++i) {
		QDomElement body_node = doc.createElement("body");
		body_node.setAttribute("id1", bodies[i].pivot1);
		body_node.setAttribute("id2", bodies[i].pivot2);
		bodies_node.appendChild(body_node);

		// setup rotation matrix
		glm::dvec2 dir = joints[bodies[i].pivot2]->pos - joints[bodies[i].pivot1]->pos;
		double angle = atan2(dir.y, dir.x);
		glm::dvec2 p1 = (joints[bodies[i].pivot1]->pos + joints[bodies[i].pivot2]->pos) * 0.5;
		glm::dmat4x4 model;
		model = glm::rotate(model, angle, glm::dvec3(0, 0, 1));

		for (int k = 0; k < bodies[i].points.size(); ++k) {
		// convert the coordinates to the local coordinate system
		glm::dvec2 rotated_p = glm::dvec2(model * glm::vec4(bodies[i].points[k].x, bodies[i].points[k].y, 0, 1)) + p1;
		QDomElement point_node = doc.createElement("point");
		point_node.setAttribute("x", rotated_p.x);
		point_node.setAttribute("y", rotated_p.y);
		body_node.appendChild(point_node);
		}
		}

		QTextStream out(&file);
		doc.save(out, 4);
		*/
	}

	void KinematicDiagram::updateBodyAdjacency() {
		// clear the neighbors
		for (int i = 0; i < bodies.size(); ++i) {
			bodies[i]->neighbors.clear();
		}

		// check the adjacency
		for (int i = 0; i < bodies.size(); ++i) {
			for (int j = i + 1; j < bodies.size(); ++j) {
				if (polygonPolygonIntersection(bodies[i]->getActualPoints(), bodies[j]->getActualPoints())) {
					bodies[i]->neighbors[j] = true;
					bodies[j]->neighbors[i] = true;
				}
			}
		}
	}

	bool KinematicDiagram::isCollided() const {
		for (int i = 0; i < bodies.size(); ++i) {
			for (int j = i + 1; j < bodies.size(); ++j) {
				// skip the neighbors
				if (bodies[i]->neighbors.contains(j)) continue;

				if (polygonPolygonIntersection(bodies[i]->getActualPoints(), bodies[j]->getActualPoints())) {
					return true;
				}
			}
		}

		return false;
	}

	void KinematicDiagram::draw(QPainter& painter, const QPointF& origin, float scale, bool show_bodies, bool show_links) const {
		if (show_bodies) {
			for (int i = 0; i < bodies.size(); ++i) {
				bodies[i]->draw(painter, origin, scale);
			}
		}

		if (show_links) {
			// draw links
			for (int i = 0; i < links.size(); ++i) {
				links[i]->draw(painter, origin, scale);
			}

			// draw joints
			for (auto it = joints.begin(); it != joints.end(); ++it) {
				joints[it.key()]->draw(painter, origin, scale);
			}
		}
	}

}