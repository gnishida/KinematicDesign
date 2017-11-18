#pragma once

#include <QString>
#include <boost/shared_ptr.hpp>
#include "kinematics.h"

namespace kinematics {

	class SCADExporter {
	protected:
		SCADExporter();

	public:
		static void save(const QString& filename, const QString& name, boost::shared_ptr<kinematics::BodyGeometry> body);
		static void save(const QString& filename, const QString& name, const std::vector<glm::dvec2>& pts, const std::vector<std::vector<glm::dvec2>>& holes, double height);
	};

}