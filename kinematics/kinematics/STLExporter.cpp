#include "STLExporter.h"
#include <QFile>
#include <QTextStream>

namespace kinematics {

	STLExporter::STLExporter() {
	}

	void STLExporter::save(const QString& filename, const QString& name, const std::vector<Vertex>& vertices) {
		QFile file(filename);
		file.open(QIODevice::WriteOnly);
		QTextStream out(&file);
		out << "solid " << name << endl;
		for (int i = 0; i < vertices.size(); i += 3) {
			out << "facet normal " << vertices[i].normal.x << " " << vertices[i].normal.y << " " << vertices[i].normal.z << endl;
			out << "  outer loop" << endl;

			for (int j = 0; j < 3; j++) {
				out << "    vertex " << vertices[i + j].position.x << " " << vertices[i + j].position.y << " " << vertices[i + j].position.z << endl;
			}

			out << "  endloop" << endl;
			out << "endfacet" << endl;
		}
		out << "endsolid " << name << endl;
		file.close();
	}

}