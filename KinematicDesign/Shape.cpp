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

	QTransform Shape::getQTransform() const {
		QTransform trans;
		trans.setMatrix(model_mat[0][0], model_mat[0][1], model_mat[0][3], model_mat[1][0], model_mat[1][1], model_mat[1][3], model_mat[3][0], model_mat[3][1], model_mat[3][3]);
		return trans;
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
	}

	void Shape::rotate(double angle) {
		glm::dvec2 c = boundingBox().center();

		model_mat = glm::translate(model_mat, glm::dvec3(c, 0));
		model_mat = glm::rotate(model_mat, angle, glm::dvec3(0, 0, 1));
		model_mat = glm::translate(model_mat, glm::dvec3(-c, 0));
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