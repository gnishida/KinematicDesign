#pragma once

#include <vector>
#include <qstring.h>
#include "Vertex.h"

namespace kinematics {

	class STLExporter {
	protected:
		STLExporter();

	public:
		static void save(const QString& filename, const QString& name, const std::vector<Vertex>& vertices);
	};

}