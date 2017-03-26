#pragma once

#include <glm/glm.hpp>

namespace canvas {

	class Operation {
	public:
		glm::dvec2 pivot;
	};

	class MoveOperation : public Operation {
	public:
		MoveOperation(const glm::dvec2& pivot);
	};

	class RotateOperation : public Operation {
	public:
		glm::dvec2 rotation_center;

	public:
		RotateOperation(const glm::dvec2& pivot, const glm::dvec2& rotation_center);
	};

	class ResizeOperation : public Operation {
	public:
		static enum { RESIZE_TOP_LEFT = 0, RESIZE_TOP_RIGHT, RESIZE_BOTTOM_LEFT, RESIZE_BOTTOM_RIGHT };

	public:
		int type;
		glm::dvec2 resize_center;

	public:
		ResizeOperation(const glm::dvec2& pivot, const glm::dvec2& resize_center);
	};

}