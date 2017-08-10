#include "History.h"

namespace canvas {

	History::History() {
		index = -1;
	}

	void History::push(std::vector<Layer> layers) {
		// remove the index-th element and their after
		history.resize(index + 1);

		// add history
		std::vector<Layer> copied_layers;
		for (int i = 0; i < layers.size(); ++i) {
			copied_layers.push_back(layers[i].clone());
		}
		history.push_back(copied_layers);
		index++;
	}

	std::vector<Layer> History::undo() {
		if (index <= 0) throw "No history.";

		// return the previous state
		index--;
		std::vector<Layer> copied_layers2;
		for (int i = 0; i < history[index].size(); ++i) {
			copied_layers2.push_back(history[index][i].clone());
		}
		return copied_layers2;
	}

	std::vector<Layer> History::redo() {
		if (index >= history.size() - 1) throw "No history.";

		index++;
		std::vector<Layer> copied_layers;
		for (int i = 0; i < history[index].size(); ++i) {
			copied_layers.push_back(history[index][i].clone());
		}
		return copied_layers;
	}

}