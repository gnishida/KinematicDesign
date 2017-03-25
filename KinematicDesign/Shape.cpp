#include "Shape.h"
#include <QImage>
#include "Utils.h"

namespace canvas {

	QImage Shape::rotation_marker = QImage("resources/rotation_marker.png").scaled(16, 16);

	Shape::Shape() {
		selected = false;
		currently_drawing = false;
		model_mat = glm::dmat4x4();
	}
	
	Shape::~Shape() {
	}

	void Shape::loadModelMat(QDomNode& node) {
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				QString name = QString("m%1%2").arg(i).arg(j);
				model_mat[i][j] = node.toElement().attribute(name.toUtf8().constData()).toDouble();
			}
		}
	}

	void Shape::loadTransform(QDomNode& node) {
		double m11 = node.toElement().attribute("m11").toDouble();
		double m12 = node.toElement().attribute("m12").toDouble();
		double m13 = node.toElement().attribute("m13").toDouble();
		double m21 = node.toElement().attribute("m21").toDouble();
		double m22 = node.toElement().attribute("m22").toDouble();
		double m23 = node.toElement().attribute("m23").toDouble();
		double m31 = node.toElement().attribute("m31").toDouble();
		double m32 = node.toElement().attribute("m32").toDouble();
		double m33 = node.toElement().attribute("m33").toDouble();

		transform.setMatrix(m11, m12, m13, m21, m22, m23, m31, m32, m33);
	}

	QDomElement Shape::toModelMatXml(QDomDocument& doc) const {
		QDomElement model_mat_node = doc.createElement("model_mat");
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				QString name = QString("m%1%2").arg(i).arg(j);
				model_mat_node.setAttribute(name.toUtf8().constData(), model_mat[i][j]);
			}
		}

		return model_mat_node;
	}

	QDomElement Shape::toTransformXml(QDomDocument& doc) const {
		QDomElement transform_node = doc.createElement("transform");
		transform_node.setAttribute("m11", transform.m11());
		transform_node.setAttribute("m12", transform.m12());
		transform_node.setAttribute("m13", transform.m13());
		transform_node.setAttribute("m21", transform.m21());
		transform_node.setAttribute("m22", transform.m22());
		transform_node.setAttribute("m23", transform.m23());
		transform_node.setAttribute("m31", transform.m31());
		transform_node.setAttribute("m32", transform.m32());
		transform_node.setAttribute("m33", transform.m33());

		return transform_node;
	}

	void Shape::select() {
		selected = true;
	}

	void Shape::unselect() {
		selected = false;
	}

	bool Shape::isSelected() const {
		return selected;
	}

	void Shape::startDrawing() {
		currently_drawing = true;
	}

	void Shape::completeDrawing() {
		currently_drawing = false;
	}

	void Shape::translate(const glm::dvec2& vec) {
		model_mat = glm::translate(glm::dmat4x4(), glm::dvec3(vec, 0)) * model_mat;

		QTransform tr;
		tr.translate(vec.x, vec.y);
		transform = transform * tr;
	}

	void Shape::rotate(double angle) {
		glm::dvec2 c = boundingBox().center();

		model_mat = glm::translate(model_mat, glm::dvec3(c, 0));
		model_mat = glm::rotate(model_mat, angle, glm::dvec3(0, 0, 1));
		model_mat = glm::translate(model_mat, glm::dvec3(-c, 0));
		transform.translate(c.x, c.y);
		transform.rotate(angle / kinematics::M_PI * 180);
		transform.translate(-c.x, -c.y);
	}

	glm::dvec2 Shape::getCenter() const {
		return boundingBox().center();
	}

	glm::dvec2 Shape::getRotationMarkerPosition() const {
		BoundingBox bbox = boundingBox();

		return glm::dvec2(bbox.center().x, bbox.minPt.y - 10);
	}
	
	glm::dvec2 Shape::localCoordinate(const glm::dvec2& point) const {
		return glm::dvec2(glm::inverse(model_mat) * glm::dvec4(point, 0, 1));
	}

	glm::dvec2 Shape::worldCoordinate(const glm::dvec2& point) const {
		return glm::dvec2(model_mat * glm::dvec4(point, 0, 1));
	}
}