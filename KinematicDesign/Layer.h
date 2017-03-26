#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <boost/shared_ptr.hpp>
#include "Shape.h"
#include <QDomDocument>

namespace canvas {

	class Layer {
	public:
		std::vector<boost::shared_ptr<Shape>> shapes;

	public:
		Layer clone() const;
		void load(QDomElement& node);
		void clear();
		void selectAll();
		void unselectAll();
		void deleteSelectedShapes();
		void copySelectedShapes(std::vector<boost::shared_ptr<Shape>>& copied_shapes);
		void pasteCopiedShapes(std::vector<boost::shared_ptr<Shape>>& copied_shapes);
		QDomElement toXml(QDomDocument& doc);
	};

}