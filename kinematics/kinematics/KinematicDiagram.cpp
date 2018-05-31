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

	Options* Options::instance = NULL;

	KinematicDiagram::KinematicDiagram() {
	}

	KinematicDiagram::~KinematicDiagram() {
	}

	KinematicDiagram KinematicDiagram::clone() const {
		KinematicDiagram copied_diagram;

		// copy joints
		for (int i = 0; i < joints.size(); ++i) {
			if (joints[i]->type == Joint::TYPE_PIN) {
				boost::shared_ptr<Joint> joint = boost::shared_ptr<Joint>(new PinJoint(joints[i]->id, joints[i]->ground, joints[i]->pos, joints[i]->z));
				joint->determined = joints[i]->determined;
				copied_diagram.addJoint(joint);
			}
			else if (joints[i]->type == Joint::TYPE_SLIDER_HINGE) {
				boost::shared_ptr<Joint> joint = boost::shared_ptr<Joint>(new SliderHinge(joints[i]->id, joints[i]->ground, joints[i]->pos, joints[i]->z));
				joint->determined = joints[i]->determined;
				copied_diagram.addJoint(joint);
			}
			else if (joints[i]->type == Joint::TYPE_GEAR) {
				boost::shared_ptr<Gear> gear = boost::static_pointer_cast<Gear>(joints[i]);
				boost::shared_ptr<Joint> joint = boost::shared_ptr<Joint>(new Gear(gear->id, gear->ground, gear->pos, gear->radius, gear->speed, gear->phase, gear->z));
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

			copied_diagram.addLink(links[i]->driver, copied_joints, links[i]->actual_link, links[i]->z);
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
			boost::shared_ptr<BodyGeometry> body = boost::shared_ptr<BodyGeometry>(new BodyGeometry(copied_diagram.joints[id1], copied_diagram.joints[id2], bodies[i]->polygons));
			/*
			for (int j = 0; j < bodies[i]->polygon.points.size(); ++j) {
			body->polygon.points.push_back(bodies[i]->polygon.points[j]);
			}
			*/

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

	boost::shared_ptr<Link> KinematicDiagram::newLink(bool driver, bool actual_link, double z) {
		int id = 0;
		if (!links.empty()) {
			id = links.lastKey() + 1;
		}

		boost::shared_ptr<Link> link = boost::shared_ptr<Link>(new Link(id, driver, actual_link, z));
		links[id] = link;
		return link;
	}

	boost::shared_ptr<Link> KinematicDiagram::addLink(boost::shared_ptr<Joint> joint1, boost::shared_ptr<Joint> joint2, bool actual_link, double z) {
		return addLink(false, joint1, joint2, actual_link, z);
	}

	boost::shared_ptr<Link> KinematicDiagram::addLink(bool driver, boost::shared_ptr<Joint> joint1, boost::shared_ptr<Joint> joint2, bool actual_link, double z) {
		int id = 0;
		if (!links.empty()) {
			id = links.lastKey() + 1;
		}

		boost::shared_ptr<Link> link = boost::shared_ptr<Link>(new Link(id, driver, actual_link, z));
		links[id] = link;

		link->addJoint(joint1);
		link->addJoint(joint2);
		joint1->links.push_back(link);
		joint2->links.push_back(link);

		return link;
	}

	boost::shared_ptr<Link> KinematicDiagram::addLink(std::vector<boost::shared_ptr<Joint>> joints, bool actual_link, double z) {
		return addLink(false, joints, actual_link, z);
	}

	boost::shared_ptr<Link> KinematicDiagram::addLink(bool driver, std::vector<boost::shared_ptr<Joint>> joints, bool actual_link, double z) {
		int id = 0;
		if (!links.empty()) {
			id = links.lastKey() + 1;
		}

		boost::shared_ptr<Link> link = boost::shared_ptr<Link>(new Link(id, driver, actual_link, z));
		links[id] = link;

		for (int i = 0; i < joints.size(); ++i) {
			link->addJoint(joints[i]);
			joints[i]->links.push_back(link);
		}

		return link;
	}

	void KinematicDiagram::addBody(boost::shared_ptr<Joint> joint1, boost::shared_ptr<Joint> joint2, const Object25D& polygons) {
		boost::shared_ptr<BodyGeometry> body = boost::shared_ptr<BodyGeometry>(new BodyGeometry(joint1, joint2, polygons));

		// get the world to local matrix
		glm::dmat3x2 model = body->getWorldToLocalModel();

		for (int i = 0; i < polygons.size(); i++) {
			for (int j = 0; j < polygons[i].points.size(); ++j) {
				// convert the coordinates to the local coordinate system
				body->polygons[i].points[j] = model * glm::dvec3(polygons[i].points[j], 1);
			}

			// Some polygon may have a top face different from the bottom face.
			// In that case, the top face polygon is stored in Polygon25D::points2.
			// Thus, we need to convert the coordinates of points2 as well.
			for (int j = 0; j < polygons[i].points2.size(); ++j) {
				// convert the coordinates to the local coordinate system
				body->polygons[i].points2[j] = model * glm::dvec3(polygons[i].points2[j], 1);
			}
		}

		bodies.push_back(body);
	}

	void KinematicDiagram::addPolygonToBody(int body_id, const Polygon25D& polygon) {
		bodies[body_id]->polygons.push_back(polygon);

		// setup rotation matrix
		glm::vec2 dir = bodies[body_id]->pivot2->pos - bodies[body_id]->pivot1->pos;
		double angle = atan2(dir.y, dir.x);

		glm::dvec2 p1 = bodies[body_id]->pivot1->pos;
		glm::dmat4x4 model;
		model = glm::rotate(model, -angle, glm::dvec3(0, 0, 1));

		for (int i = 0; i < polygon.points.size(); i++) {
			// convert the coordinates to the local coordinate system
			glm::dvec2 rotated_p = glm::dvec2(model * glm::dvec4(polygon.points[i].x - p1.x, polygon.points[i].y - p1.y, 0, 1));

			bodies[body_id]->polygons.back().points[i] = rotated_p;
		}

		// Somd polygon may have a top face different from the bottom face.
		// In that case, the top face polygon is stored in Polygon25D::points2.
		// Thus, we need to convert the coordinates of points2 as well.
		for (int i = 0; i < polygon.points2.size(); i++) {
			// convert the coordinates to the local coordinate system
			glm::dvec2 rotated_p = glm::dvec2(model * glm::dvec4(polygon.points2[i].x - p1.x, polygon.points2[i].y - p1.y, 0, 1));

			bodies[body_id]->polygons.back().points2[i] = rotated_p;
		}
	}
	
	/**
	 * Given the current mechanism, already added moving bodies, and input fixed bodies,
	 * this function updates the fixed bodies and moving bodies such that
	 * all the joints are properly connected to some rigid bodies.
	 */
	void KinematicDiagram::connectJointsToBodies(std::vector<Object25D>& fixed_bodies, const std::vector<std::vector<int>>& zorder, std::vector<glm::dvec2>& connected_pts) {
		int N = fixed_bodies.size();

		// clear the connectors
		connectors.clear();

		// connect fixed joints to a fixed body
		int cnt = 0;
		for (int j = 0; j < joints.size(); j++) {
			if (joints[j]->ground) {
				glm::dvec2 connected_pt = connectFixedJointToBody(joints[j], fixed_bodies, zorder.size() == 3 ? zorder[0][cnt++] : 1);
				connected_pts.push_back(connected_pt);
			}
		}

		// connect moving joints to a moving body
		cnt = 0;
		for (int j = 0; j < bodies.size(); j++) {
			if (bodies[j]->pivot1->ground ||bodies[j]->pivot2->ground) continue;

			std::vector<glm::dvec2> body_pts = bodies[j]->getActualPoints()[0];
			glm::dvec2 connected_pt1 = connectMovingJointToBody(bodies[j]->pivot1, j, body_pts, zorder.size() == 3 ? zorder[1][cnt++] : 1);
			connected_pts.push_back(connected_pt1);
			glm::dvec2 connected_pt2 = connectMovingJointToBody(bodies[j]->pivot2, j, body_pts, zorder.size() == 3 ? zorder[1][cnt++] : 1);
			connected_pts.push_back(connected_pt2);
		}

		// add links as additional connectors
		for (int i = 0; i < links.size(); i++) {
			if (links[i]->actual_link) {
				connectors.push_back(JointConnector(links[i]->joints));
			}
		}

		// check the adjacency between link and connector
		for (int i = 0; i < connectors.size(); i++) {
			for (int j = i + 1; j < connectors.size(); j++) {
				for (int k = 0; k < connectors[i].joints.size(); k++) {
					for (int l = 0; l < connectors[j].joints.size(); l++) {
						if (connectors[i].joints[k] == connectors[j].joints[l]) {
							connectors[i].neighbors[j] = true;
							connectors[j].neighbors[i] = true;
						}
					}
				}
			}
		}
	}

	/**
	 * Connect fixed joint to a body by adding a connector
	 * Joint has a information of z order that indicates the z order of the connected link.
	 * Based on the z orders of the joint and joint connector, the direction of the joint, upward or downward, can be determined.
	 *
	 * @param joint	joint
	 * @param z		z order of the joint connector
	 */
	glm::dvec2 KinematicDiagram::connectFixedJointToBody(boost::shared_ptr<kinematics::Joint> joint, std::vector<Object25D>& fixed_bodies, double link_z) {
		glm::dvec2 closest_point;
		int fixed_body_id = -1;

		// check if the joint is within the rigid body
		bool is_inside = false;
		for (int k = 0; k < fixed_bodies.size(); k++) {
			if (kinematics::withinPolygon(fixed_bodies[k].polygons[0].points, joint->pos)) {
				is_inside = true;
				fixed_body_id = k;
				break;
			}
		}

		std::vector<glm::dvec2> pts;
		double z = 0;
		double height = 0;

		if (is_inside) {
			closest_point = joint->pos;

			// create connector object
			connectors.push_back(JointConnector(joint, joint->pos));

			pts = generateCirclePolygon(joint->pos, options->link_width / 2);
			z = 0;
			height = link_z * (options->link_depth + options->joint_cap_depth + options->gap * 2);
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts, z, z + height));
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts, -options->body_depth - z - height, -options->body_depth - z));
		}
		else {
			// find the closest point of a rigid body
			double min_dist = std::numeric_limits<double>::max();
			for (int k = 0; k < fixed_bodies.size(); k++) {
				glm::dvec2 cp;
				try {
					cp = kinematics::closestOffsetPoint(fixed_bodies[k].polygons[0].points, joint->pos, options->body_margin);
				}
				catch (char* ex) {
					double dist;
					cp = kinematics::closestPoint(fixed_bodies[k].polygons[0].points, joint->pos, dist);
				}

				double dist = glm::length(cp - joint->pos);
				if (dist < min_dist) {
					min_dist = dist;
					closest_point = cp;
					fixed_body_id = k;
				}
			}

			// create connector object
			connectors.push_back(JointConnector(joint, closest_point));

			// Create the base of the connecting part
			pts = generateCirclePolygon(closest_point, options->link_width / 2);
			z = 0;
			height = link_z * (options->link_depth + options->joint_cap_depth + options->gap * 2) - options->link_depth;
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts, z, z + height));
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts, -options->body_depth - z - height, -options->body_depth - z));

			// Create a link-like geometry to extend the body to the joint
			z += height;
			height = options->link_depth;
			pts = generateRoundedBarPolygon(closest_point, joint->pos, options->link_width / 2);
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts, z, z + height));
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts, -options->body_depth - z - height, -options->body_depth - z));
		}

		if (joint->z > link_z) {
			// Create the joint part of the body
			// First, create the base
			pts = generateCirclePolygon(joint->pos, options->link_width / 2);
			z += height;
			height = (joint->z - link_z) * (options->link_depth + options->gap * 2 + options->joint_cap_depth) - options->link_depth - options->gap;
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts, z, z + height));
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts, -options->body_depth - z - height, -options->body_depth - z));

			// Second, create the rod of the joint
			pts = generateCirclePolygon(joint->pos, options->joint_radius);
			z += height;
			height = options->link_depth + options->gap * 2;
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts, z, z + height, false));
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts, -options->body_depth - z - height, -options->body_depth - z, false));

			// Finally, create the cap of the joint
			// For the cap, the top face is a little smaller, so we use a different polygon for that.
			pts = generateCirclePolygon(joint->pos, options->joint_cap_radius1);
			std::vector<glm::dvec2> pts2 = generateCirclePolygon(joint->pos, options->joint_cap_radius2);
			z += height;
			height = options->joint_cap_depth;
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts2, pts, z, z + height, false));
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts, pts2, -options->body_depth - z - height, -options->body_depth - z, false));
		}
		else if (joint->z < link_z) {
			// Create the joint part of the body
			// First, create the base
			pts = generateCirclePolygon(joint->pos, options->link_width / 2);
			height = (link_z - joint->z) * (options->link_depth + options->gap * 2 + options->joint_cap_depth) - options->link_depth - options->gap;
			z -= height;
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts, z, z + height));
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts, -options->body_depth - z - height, -options->body_depth - z));

			// Second, create the rod of the joint
			pts = generateCirclePolygon(joint->pos, options->joint_radius);
			height = options->link_depth + options->gap * 2;
			z -= height;
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts, z, z + height, false));
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts, -options->body_depth - z - height, -options->body_depth - z, false));

			// Finally, create the cap of the joint
			// For the cap, the top face is a little smaller, so we use a different polygon for that.
			pts = generateCirclePolygon(joint->pos, options->joint_cap_radius1);
			std::vector<glm::dvec2> pts2 = generateCirclePolygon(joint->pos, options->joint_cap_radius2);
			height = options->joint_cap_depth;
			z -= height;
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts, pts2, z, z + height, false));
			fixed_bodies[fixed_body_id].push_back(kinematics::Polygon25D(pts2, pts, -options->body_depth - z - height, -options->body_depth - z, false));
		}

		return closest_point;
	}

	glm::dvec2 KinematicDiagram::connectMovingJointToBody(boost::shared_ptr<Joint> joint, int body_id, const std::vector<glm::dvec2>& moving_body, double link_z) {
		glm::dvec2 closest_point;
		std::vector<glm::dvec2> pts;
		double z = 0;
		double height = 0;

		if (kinematics::withinPolygon(moving_body, joint->pos)) {
			closest_point = joint->pos;

			// create connector object
			connectors.push_back(JointConnector(joint, bodies[body_id]->worldToLocal(joint->pos), bodies[body_id]));

			pts = generateCirclePolygon(joint->pos, options->link_width / 2);
			z = 0;
			height = link_z * (options->link_depth + options->joint_cap_depth + options->gap * 2);
			addPolygonToBody(body_id, kinematics::Polygon25D(pts, z, z + height));
			addPolygonToBody(body_id, kinematics::Polygon25D(pts, -options->body_depth - z - height, -options->body_depth - z));
		}
		else {
			// find the closest point of a rigid body
			try {
				closest_point = kinematics::closestOffsetPoint(moving_body, joint->pos, options->body_margin);
			}
			catch (char* ex) {
				double dist;
				closest_point = kinematics::closestPoint(moving_body, joint->pos, dist);
			}

			// create connector object
			connectors.push_back(JointConnector(joint, bodies[body_id]->worldToLocal(closest_point), bodies[body_id]));

			// Create the base of the connecting part
			pts = generateCirclePolygon(closest_point, options->link_width / 2);
			z = 0;
			height = link_z * (options->link_depth + options->joint_cap_depth + options->gap * 2) - options->link_depth;
			addPolygonToBody(body_id, kinematics::Polygon25D(pts, z, z + height));
			addPolygonToBody(body_id, kinematics::Polygon25D(pts, -options->body_depth - z - height, -options->body_depth - z));

			// create a geometry to extend the body to the joint
			pts = generateRoundedBarPolygon(closest_point, joint->pos, options->link_width / 2);
			z += height;
			height = options->link_depth;
			addPolygonToBody(body_id, kinematics::Polygon25D(pts, z, z + height));
			addPolygonToBody(body_id, kinematics::Polygon25D(pts, -options->body_depth - z - height, -options->body_depth - z));
		}

		if (joint->z > link_z) {
			// Create the joint part of the body
			// First, create the base
			pts = generateCirclePolygon(joint->pos, options->link_width / 2);
			z += height;
			height = (joint->z - link_z) * (options->link_depth + options->gap * 2 + options->joint_cap_depth) - options->link_depth - options->gap;
			addPolygonToBody(body_id, kinematics::Polygon25D(pts, z, z + height));
			addPolygonToBody(body_id, kinematics::Polygon25D(pts, -options->body_depth - z - height, -options->body_depth - z));

			// Second, create the rod of the joint
			pts = generateCirclePolygon(joint->pos, options->joint_radius);
			z += height;
			height = options->link_depth + options->gap * 2;
			addPolygonToBody(body_id, kinematics::Polygon25D(pts, z, z + height, false));
			addPolygonToBody(body_id, kinematics::Polygon25D(pts, -options->body_depth - z - height, -options->body_depth - z, false));

			// Finally, create the cap of the joint
			// For the cap, the top face is a little smaller, so we use a different polygon for that.
			pts = generateCirclePolygon(joint->pos, options->joint_cap_radius1);
			std::vector<glm::dvec2> pts2 = generateCirclePolygon(joint->pos, options->joint_cap_radius2);
			z += height;
			height = options->joint_cap_depth;
			addPolygonToBody(body_id, kinematics::Polygon25D(pts2, pts, z, z + height, false));
			addPolygonToBody(body_id, kinematics::Polygon25D(pts, pts2, -options->body_depth - z - height, -options->body_depth - z, false));
		}
		else if (joint->z < link_z) {
			// Create the joint part of the body
			// First, create the base
			pts = generateCirclePolygon(joint->pos, options->link_width / 2);
			height = (link_z - joint->z) * (options->link_depth + options->gap * 2 + options->joint_cap_depth) - options->link_depth - options->gap;
			z -= height;
			addPolygonToBody(body_id, kinematics::Polygon25D(pts, z, z + height));
			addPolygonToBody(body_id, kinematics::Polygon25D(pts, -options->body_depth - z - height, -options->body_depth - z));

			// Second, create the rod of the joint
			pts = generateCirclePolygon(joint->pos, options->joint_radius);
			height = options->link_depth + options->gap * 2;
			z -= height;
			addPolygonToBody(body_id, kinematics::Polygon25D(pts, z, z + height, false));
			addPolygonToBody(body_id, kinematics::Polygon25D(pts, -options->body_depth - z - height, -options->body_depth - z, false));

			// Finally, create the cap of the joint
			// For the cap, the top face is a little smaller, so we use a different polygon for that.
			pts = generateCirclePolygon(joint->pos, options->joint_cap_radius1);
			std::vector<glm::dvec2> pts2 = generateCirclePolygon(joint->pos, options->joint_cap_radius2);
			height = options->joint_cap_depth;
			z -= height;
			addPolygonToBody(body_id, kinematics::Polygon25D(pts, pts2, z, z + height, false));
			addPolygonToBody(body_id, kinematics::Polygon25D(pts2, pts, -options->body_depth - z - height, -options->body_depth - z, false));
		}

		return closest_point;
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
						double depth1 = body_node.toElement().attribute("depth1").toDouble();
						double depth2 = body_node.toElement().attribute("depth2").toDouble();

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

						addBody(joints[id1], joints[id2], Object25D(points, depth1, depth2));
					}

					body_node = body_node.nextSibling();
				}
			}

			node = node.nextSibling();
		}

		// initialize the adancency between rigid bodies
		initialize();
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

		// Check the adjacency
		// The current implementation is not elegant.
		// Adjacent bodies are considered to belong to the same group of a fixed body,
		// and we do not check the collision between them.
		for (int i = 0; i < bodies.size(); ++i) {
			for (int j = i + 1; j < bodies.size(); ++j) {
				if (polygonPolygonIntersection(bodies[i]->getActualPoints()[0], bodies[j]->getActualPoints()[0])) {
					bodies[i]->neighbors[j] = true;
					bodies[j]->neighbors[i] = true;
				}
			}
		}
	}

	/**
	 * Check the collision between bodies.
	 * If main_body_only is true, check only the main bodies, and the joint connectors are not checked.
	 */
	bool KinematicDiagram::isCollided(bool main_body_only) const {
		// check the collision between rigid bodies
		for (int i = 0; i < bodies.size(); ++i) {
			for (int j = i + 1; j < bodies.size(); ++j) {
				// skip the neighbors
				if (bodies[i]->neighbors.contains(j)) continue;

				for (int k = 0; k < bodies[i]->polygons.size(); k++) {
					if (main_body_only && k > 0) continue;

					if (!bodies[i]->polygons[k].check_collision) continue;
					std::vector<glm::dvec2> pts1 = bodies[i]->getActualPoints(k);

					for (int l = 0; l < bodies[j]->polygons.size(); l++) {
						if (main_body_only && l > 0) continue;

						if (!bodies[j]->polygons[l].check_collision) continue;
						std::vector<glm::dvec2> pts2 = bodies[j]->getActualPoints(l);

						// if the depth is different, we do not need to check the collision between these rigid bodies
						if (bodies[i]->polygons[k].depth1 >= bodies[j]->polygons[l].depth2 || bodies[j]->polygons[l].depth1 >= bodies[i]->polygons[k].depth2) continue;

						if (polygonPolygonIntersection(pts1, pts2)) return true;
					}
				}
			}
		}

		// This check is needed strictly speaking, but
		// the collision that this check detects should not happen
		// because the appropriate z-orders are selected.
		/*if (!main_body_only) {
			// check the collision between links and joints
			for (int i = 0; i < links.size(); i++) {
				// For the coupler, we can use the moving body itself as a coupler, 
				// so we do not need to create a coupler link.
				if (!links[i]->actual_link) continue;

				for (int j = 0; j < joints.size(); j++) {
					// If the link i is furhter away from the body than joint j,
					// there will be no collision between them.
					if (links[i]->z > joints[j]->z) continue;

					// If joint j belongs to link i, skip it for collision check.
					bool excluded = false;
					for (int k = 0; k < links[i]->joints.size(); k++) {
						if (joints[j] == links[i]->joints[k]) {
							excluded = true;
							break;
						}
					}
					if (excluded) continue;

					for (int k = 0; k < links[i]->joints.size(); k++) {
						for (int l = k + 1; l < links[i]->joints.size(); l++) {
							if (distanceToSegment(links[i]->joints[k]->pos, links[i]->joints[l]->pos, joints[j]->pos) < options->link_width) {
								return true;
							}
						}
					}
				}
			}
		}*/

		return false;
	}

	/**
	 * Record collision between links, connectors and joints.
	 * This information is then used for z-ordering the links and connectors.
	 */
	void KinematicDiagram::recordCollisionForConnectors() {
		// check if connector i collide with joints of connector j
		for (int i = 0; i < connectors.size(); i++) {
			glm::dvec2 pt1a = connectors[i].joints[0]->pos;
			glm::dvec2 pt1b = connectors[i].closest_pt;
			if (connectors[i].type == 1) {
				pt1b = connectors[i].body->localToWorld(connectors[i].closest_pt);
			}
			else if (connectors[i].type == 2) {
				// pt1b = connectors[i].joints[1]->pos;
				// Bug Fixed!
				// Since the guide of the slider crank has three joints, [0] one end of the guide, [1] slider, and [2] the other end of the guide,
				// we have to use [0] as pt1a and [2] as pt1b to represent the guide.
				pt1b = connectors[i].joints.back()->pos;
			}

			for (int j = 0; j < connectors.size(); j++) {
				if (i == j) continue;

				// if both connectors are attached to the fixed body, skip the collision check.
				if (connectors[i].type == 0 && connectors[j].type == 0) continue;

				// if both connectors are attached to the same moving body, skip the collision check.
				if (connectors[i].type == 1 && connectors[j].type == 1 && connectors[i].body == connectors[j].body) continue;
				



				// check the collision for the connector base of connector j, which is case 1.
				if (connectors[j].type == 0 || connectors[j].type == 1) {
					glm::dvec2 pt2b = connectors[j].closest_pt;
					if (connectors[j].type == 1) {
						pt2b = connectors[j].body->localToWorld(connectors[j].closest_pt);
					}
					if (distanceToSegment(pt1a, pt1b, pt2b) < options->link_width) {
						connectors[i].collisions1[j] = true;
					}
				}

				// check the collision for the joint of connector j, which is case 2.
				for (int k = 0; k < connectors[j].joints.size(); k++) {
					if (connectors[i].hasJoint(connectors[j].joints[k])) continue;

					// check the collision for k-th joint of connector j, which is case 2.
					if (distanceToSegment(pt1a, pt1b, connectors[j].joints[k]->pos) < options->link_width) {
						bool same_body = false;
						std::vector<int> list;
						list.push_back(j);
						for (int l = 0; l < connectors.size(); l++) {
							if (l == j || l == i) continue;
							if (connectors[l].hasJoint(connectors[j].joints[k])) {
								list.push_back(l);
								if (connectors[i].type == 0 && connectors[l].type == 0) same_body = true;
								if (connectors[i].type == 1 && connectors[l].type == 1 && connectors[i].body == connectors[l].body) same_body = true;
							}
						}
						if (!same_body) {
							connectors[i].collisions2[connectors[j].joints[k]->id] = list;
						}
					}
				}
			}
		}
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