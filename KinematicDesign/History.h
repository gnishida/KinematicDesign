#pragma once

#include <vector>
#include "Design.h"

namespace canvas {

	class History {
	private:
		int index;
		std::vector<Design> history;

	public:
		History();

		void push(Design design);
		Design undo();
		Design redo();
	};

}