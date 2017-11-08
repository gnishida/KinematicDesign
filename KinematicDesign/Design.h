#pragma once

#include <vector>
#include <boost/shared_ptr.hpp>
#include "Shape.h"
#include "RenderManager.h"

namespace canvas {

	class MovingBody {
	public:
		boost::shared_ptr<Shape> linkage_region;
		boost::shared_ptr<Shape> linkage_avoidance;
		std::vector<boost::shared_ptr<Shape>> poses;

	public:
		MovingBody clone() const;
		void load(QDomNode& node);
		QDomElement toXml(QDomDocument& doc);
	};

	class Design {
	public:
		int num_layers;
		int layer_id;
		std::vector<boost::shared_ptr<Shape>> fixed_bodies;
		std::vector<MovingBody> moving_bodies;

		// copied shapes
		std::vector<boost::shared_ptr<Shape>> copied_fixed_bodies;

	public:
		Design();

		Design clone() const;
		void addMovingBody(boost::shared_ptr<Shape> shape);
		void load(const QString& filename);
		void save(const QString& filename);
		void clear();
		void selectAll();
		void unselectAll();
		void deleteSelectedShapes();
		void copySelectedShapes();
		void pasteCopiedShapes();
		void addLayer();
		void insertLayer();
		bool deleteLayer();

		void generate3DGeometry(RenderManager& renderManager) const;

		bool hitTest(const glm::dvec2& pt, bool multiple_selection, boost::shared_ptr<Shape>& selected_shape);
		bool hitTestRotationMarker(const glm::dvec2& pt, double scale, double threshold, boost::shared_ptr<Shape>& selected_shape, glm::dvec2& rotate_pivot);
		bool hitTestResizeMarker(const glm::dvec2& pt, double threshold, boost::shared_ptr<Shape>& selected_shape, glm::dvec2& resize_pivot);
		void move(const glm::dvec2& dir, RenderManager& renderManager);
		void rotate(double theta, RenderManager& renderManager);
		void resize(const glm::dvec2& resize_scale, const glm::dvec2& resize_center, RenderManager& renderManager);
	};

}