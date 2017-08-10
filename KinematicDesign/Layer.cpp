#include "Layer.h"
#include "Rectangle.h"
#include "Circle.h"
#include "Polygon.h"

namespace canvas {

	Layer Layer::clone() const {
		Layer copied_layer;
		for (int i = 0; i < shapes.size(); ++i) {
			copied_layer.shapes.push_back(shapes[i]->clone());
		}
		return copied_layer;
	}

	void Layer::load(QDomElement& node) {
		QDomNode shape_node = node.firstChild();
		while (!shape_node.isNull()) {
			if (shape_node.toElement().tagName() == "shape") {
				int subtype = shape_node.toElement().attribute("subtype").toInt();
				if (shape_node.toElement().attribute("type") == "rectangle") {
					shapes.push_back(boost::shared_ptr<Shape>(new Rectangle(subtype, shape_node)));
				}
				else if (shape_node.toElement().attribute("type") == "circle") {
					shapes.push_back(boost::shared_ptr<Shape>(new Circle(subtype, shape_node)));
				}
				else if (shape_node.toElement().attribute("type") == "polygon") {
					shapes.push_back(boost::shared_ptr<Shape>(new Polygon(subtype, shape_node)));
				}
			}

			shape_node = shape_node.nextSibling();
		}
	}

	void Layer::clear() {
		shapes.clear();
	}

	void Layer::selectAll() {
		for (int i = 0; i < shapes.size(); ++i) {
			shapes[i]->select();
		}
	}

	void Layer::unselectAll() {
		for (int i = 0; i < shapes.size(); ++i) {
			shapes[i]->unselect();
		}
	}

	void Layer::deleteSelectedShapes() {
		for (int i = shapes.size() - 1; i >= 0; --i) {
			if (shapes[i]->isSelected()) {
				shapes.erase(shapes.begin() + i);
			}
		}
	}

	void Layer::copySelectedShapes(std::vector<boost::shared_ptr<Shape>>& copied_shapes) {
		copied_shapes.clear();
		for (int i = 0; i < shapes.size(); ++i) {
			if (shapes[i]->isSelected()) {
				copied_shapes.push_back(shapes[i]->clone());
			}
		}
	}

	void Layer::pasteCopiedShapes(std::vector<boost::shared_ptr<Shape>>& copied_shapes) {
		unselectAll();
		for (int i = 0; i < copied_shapes.size(); ++i) {
			boost::shared_ptr<Shape> shape = copied_shapes[i]->clone();
			shape->select();
			shapes.push_back(shape);
		}
	}

	QDomElement Layer::toXml(QDomDocument& doc) {
		QDomElement layer_node = doc.createElement("layer");

		for (int i = 0; i < shapes.size(); ++i) {
			QDomElement shape_node = shapes[i]->toXml(doc);
			
			layer_node.appendChild(shape_node);
		}

		return layer_node;
	}

}