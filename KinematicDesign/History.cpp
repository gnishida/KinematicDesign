#include "History.h"

namespace canvas {

	History::History() {
		index = -1;
	}

	void History::push(Design design) {
		// remove the index-th element and their after
		history.resize(index + 1);

		// add history
		history.push_back(design.clone());
		index++;
	}

	Design History::undo() {
		if (index <= 0) throw "No history.";

		// return the previous state
		return history[--index].clone();
	}

	Design History::redo() {
		if (index >= history.size() - 1) throw "No history.";

		// return the next state
		return history[++index].clone();
	}

}