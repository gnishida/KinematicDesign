#pragma once

#include <vector>
#include "Layer.h"

namespace canvas {

	class History {
	private:
		int index;
		std::vector<std::vector<Layer>> history;

	public:
		History();

		void push(std::vector<Layer> layers);
		std::vector<Layer> undo();
		std::vector<Layer> redo();
	};

}