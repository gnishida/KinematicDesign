#include "Operation.h"

namespace canvas {

	MoveOperation::MoveOperation(const glm::dvec2& pivot) { 
		this->pivot = pivot;
	}

	RotateOperation::RotateOperation(const glm::dvec2& pivot, const glm::dvec2& rotation_center) {
		this->pivot = pivot;
		this->rotation_center = rotation_center;
	}

	ResizeOperation::ResizeOperation(const glm::dvec2& pivot, const glm::dvec2& resize_center) {
		this->pivot = pivot; 
		this->resize_center = resize_center;
	}
}